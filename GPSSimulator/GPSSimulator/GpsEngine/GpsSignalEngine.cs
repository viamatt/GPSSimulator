using static GPSSimulator.GpsEngine.GpsConstants;
using static GPSSimulator.GpsEngine.GpsMath;
using static GPSSimulator.GpsEngine.GpsNavMessage;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// Core GPS L1 C/A signal generator. Streams signed 8-bit IQ samples
/// directly compatible with HackRF (SC08 format).
/// 
/// The engine reads CurrentXyz at every 0.1-second epoch to support
/// live / dynamic position updates.
/// 
/// Ported from gpssim.c (osqzss/gps-sdr-sim, MIT licence).
/// </summary>
public sealed class GpsSignalEngine
{
	// ── public API ──────────────────────────────────────────────────────

	public event Action<string>? LogMessage;

	/// <summary>
	/// ECEF position in metres. Thread-safe: update at any time.
	/// </summary>
	public double[] CurrentXyz
	{
		get => Volatile.Read(ref _xyz);
		set => Volatile.Write(ref _xyz, value);
	}

	/// <summary>
	/// Optional trajectory source. When set, the engine calls this once per
	/// 0.1 s epoch with the elapsed simulation time (seconds since streaming
	/// started) and uses the returned ECEF position instead of <see cref="CurrentXyz"/>.
	///
	/// This exists because Doppler is derived from the change in position between
	/// consecutive epochs. If position is pushed in from an external timer, that
	/// timer's jitter is indistinguishable from real velocity and is injected
	/// straight into the receiver's carrier tracking loop, and the error grows
	/// with speed. Sampling the trajectory on the engine's own epoch clock makes
	/// the interval exactly 0.1 s every time, so the Doppler is exact.
	/// </summary>
	public Func<double, double[]>? PositionProvider { get; set; }

	// ── private state ────────────────────────────────────────────────

	private double[] _xyz = new double[3];

	private readonly Ephemeris[][] _eph;
	private readonly IonoUtc _ionoutc;
	private readonly ChannelState[] _chan;
	private readonly double _sampFreq;
	private readonly double _elevMask;

	/// <summary>
	/// Number of satellites actually transmitted. A receiver only needs 4 for a
	/// fix; fewer channels means each one gets a larger share of the fixed SC08
	/// dynamic range (better per-satellite C/N0) and less CPU per epoch.
	/// </summary>
	private readonly int _maxActiveChan;

	private const int MaxChan = GpsConstants.MaxChan;
	private const int MaxSat  = GpsConstants.MaxSat;
	private const int DebugLogInterval10Hz = 100; // every 10 seconds
	private const bool UseFixedPowerModel = false; // default to gps-sdr-sim-like path-loss model
	private const int FixedChannelGain = 112;      // only used when fixed-power mode is enabled
	private const int Sc08RightShift = 4;          // base SC08 scaling (matches gps.c: iq_buff[i] >> 4)
	private const int Sc08BoostNumerator = 1;      // no digital boost - parity with reference implementations
	private const int Sc08BoostDenominator = 1;

	/// <summary>
	/// Target RMS (standard deviation) of each SC08 component. The sum of N
	/// independent PRN signals is approximately Gaussian, so peaks reach roughly
	/// 4 sigma. At sigma = 24 that puts the 4-sigma peak near 96, comfortably
	/// inside +/-127, giving a fraction of a percent of clipping.
	/// Raise for more power, lower if clip% exceeds ~1%.
	/// </summary>
	private const double Sc08TargetRms = 24.0;

	/// <summary>
	/// When true, channel gains are renormalised each epoch so the composite
	/// signal reaches <see cref="Sc08TargetRms"/> regardless of how many channels
	/// are active. When false the gain chain matches gps-sdr-sim /
	/// multi-sdr-gps-sim exactly (absolute path-loss gain, plain >>4 on packing).
	/// </summary>
	private const bool UseGainNormalization = true;

	/// <summary>
	/// Hysteresis applied to the elevation mask (degrees). A satellite must rise
	/// above the mask to be acquired, but is only dropped once it falls this far
	/// below it. Without hysteresis an SV sitting near the mask flips in and out
	/// on successive reallocations, and every re-add costs the receiver a fresh
	/// acquisition, which shows up as intermittent loss of fix.
	/// </summary>
	private const double ElevMaskHysteresisDeg = 2.5;

	public GpsSignalEngine(
		Ephemeris[][] eph,
		IonoUtc ionoutc,
		double sampFreq = DefaultSampleRate,
		double elevMaskDeg = 0.0,
		int maxActiveChan = MaxChan)
	{
		_eph      = eph;
		_ionoutc  = ionoutc;
		_sampFreq = sampFreq;
		_elevMask = elevMaskDeg * Pi / 180.0;
		_maxActiveChan = Math.Clamp(maxActiveChan, 4, MaxChan);
		_chan      = new ChannelState[MaxChan];
		for (int i = 0; i < MaxChan; i++) _chan[i] = new ChannelState();
	}

	// ── Main streaming method ────────────────────────────────────────

	/// <summary>
	/// Continuously generates SC08 IQ bytes and writes them to <paramref name="output"/>.
	/// Reads CurrentXyz at every 0.1-second epoch. Stops when ct is cancelled.
	/// </summary>
	public async Task StreamAsync(Stream output, GpsTime startTime, CancellationToken ct)
	{
		double samp_freq = Math.Floor(_sampFreq / 10.0);
		int    iq_buff_size = (int)samp_freq;   // samples per 0.1 s
		samp_freq *= 10.0;
		double delt = 1.0 / samp_freq;

		// Phase step for integer carrier phase accumulator (32-bit wraps around 512)
		// step = freq / sampFreq * 512  (512 = lookup table size)
		// We compute per-channel below.

		// Antenna gain pattern (linear, from dB table)
		double[] antPat = new double[37];
		for (int i = 0; i < 37; i++)
			antPat[i] = Math.Pow(10.0, -AntPatDb[i] / 20.0);

		var grx = startTime;
		int ieph = FindBestEphemerisIndex(grx);
		int epochCount10Hz = 0;
		long clipCountAcc = 0;
		long sampleCountAcc = 0;
		int  peakAcc = 0;
		double sumSqAcc = 0.0;

		// Allocate initial channels
		double[] xyz0 = (double[])CurrentXyz.Clone();
		AllocateChannels(grx, xyz0, ieph);
		Log($"Initial ephemeris slot: ieph={ieph}");

		// IQ buffers
		short[] iq_buff   = new short[iq_buff_size * 2];
		byte[]  iq8_buff  = new byte [iq_buff_size * 2];

		// Per-epoch channel gains. gps-sdr-sim / multi-sdr-gps-sim compute the
		// gain once per 0.1 s epoch, NOT per sample. Doing it per sample costs a
		// float division + clamp for every sample of every channel and makes the
		// generator slower than real time.
		int[] chanGain = new int[MaxChan];

		// Gain normalisation factor, locked in on the first epoch with active
		// channels and then held constant. See the usage site for why.
		double gainScale = -1.0;

		// Measures pure generation cost per epoch (excludes the blocking write to
		// the SDR). Must stay well below 100 ms/epoch or TX will underrun.
		var genStopwatch = new System.Diagnostics.Stopwatch();

		Log($"GPS signal engine started. Sample rate: {samp_freq / 1e6:F2} MHz, buffer: {iq_buff_size} samples/epoch");

		while (!ct.IsCancellationRequested)
		{
			genStopwatch.Start();
			// Sample the trajectory on the engine's own epoch clock when available,
			// so consecutive positions are always exactly 0.1 s apart.
			double[] xyz = PositionProvider is { } provider
				? provider(epochCount10Hz * 0.1)
				: (double[])CurrentXyz.Clone();
			CurrentXyz = xyz;
			UpdateActiveChannelCodePhase(grx, xyz, ieph);

			int nActiveGains = 0;
			for (int i = 0; i < MaxChan; i++)
			{
				if (_chan[i].Prn == 0) { chanGain[i] = 0; continue; }
				nActiveGains++;

				if (UseFixedPowerModel)
				{
					chanGain[i] = FixedChannelGain;
				}
				else
				{
					int iang = (int)(90.0 - _chan[i].Azel[1] * R2D);
					iang = Math.Clamp(iang / 5, 0, 36); // 0:5:180 => index 0-36
					double antGain = antPat[iang];
					double pathLoss = 20200000.0 / Math.Max(_chan[i].Rho0.D, 1.0); // gps-sdr-sim style
					chanGain[i] = (int)(pathLoss * antGain * 128.0);
				}
			}

			// Gain normalisation. The scale factor is computed ONCE, from the first
			// epoch that has active channels, and then held fixed for the whole run.
			//
			// It must NOT be recomputed per epoch: the scale depends on how many
			// channels are active, so every time AllocateChannels adds or drops a
			// satellite the factor would change and rescale *every* channel at once.
			// The receiver sees a simultaneous step in C/N0 on all satellites, which
			// is a classic way to lose lock. Holding the scale fixed means adding or
			// dropping a satellite changes only that satellite's contribution.
			if (UseGainNormalization && nActiveGains > 0)
			{
				if (gainScale <= 0.0)
				{
					double sumSq = 0.0;
					for (int i = 0; i < MaxChan; i++)
					{
						double a = chanGain[i] * 250.0 / 128.0 / (1 << Sc08RightShift);
						sumSq += a * a;
					}
					if (sumSq > 0.0)
					{
						// Per-component (I or Q) standard deviation: carrier phase splits
						// each channel's power between I and Q, hence the factor of 2.
						double sigma = Math.Sqrt(sumSq / 2.0);
						gainScale = Sc08TargetRms / Math.Max(sigma, 1e-9);
						Log($"Gain normalisation locked: scale={gainScale:F4} from {nActiveGains} active channels");
					}
				}

				if (gainScale > 0.0)
				{
					for (int i = 0; i < MaxChan; i++)
						chanGain[i] = (int)(chanGain[i] * gainScale);
				}
			}

			// ── Generate one 0.1-second epoch of IQ samples ────────────
			// Hot loop: compacted to active channels only, with pinned pointers
			// to eliminate per-sample bounds checks and field indirection.
			GenerateEpoch(iq_buff, iq_buff_size, chanGain, delt);

			// Convert to SC08 for HackRF.
			// Keep per-satellite power stable as channel count changes:
			// no auto-attenuation, only saturate to avoid wraparound.
			for (int isamp = 0; isamp < iq_buff_size * 2; isamp++)
			{
				int raw = Sc08BoostNumerator == 1 && Sc08BoostDenominator == 1
					? (iq_buff[isamp] >> Sc08RightShift)   // arithmetic shift, matches gps.c
					: (iq_buff[isamp] * Sc08BoostNumerator) / (Sc08BoostDenominator << Sc08RightShift);
				int v = Math.Clamp(raw, -127, 127);
				if (v != raw) clipCountAcc++;
				int mag = v < 0 ? -v : v;
				if (mag > peakAcc) peakAcc = mag;
				sumSqAcc += (double)v * v;
				sampleCountAcc++;
				iq8_buff[isamp] = (byte)(sbyte)v;
			}

			genStopwatch.Stop();
			await output.WriteAsync(iq8_buff, 0, iq8_buff.Length, ct);

			epochCount10Hz++;

			if (epochCount10Hz % DebugLogInterval10Hz == 0)
			{
				double clipPct = sampleCountAcc > 0 ? (100.0 * clipCountAcc / sampleCountAcc) : 0.0;
				double msPerEpoch = genStopwatch.Elapsed.TotalMilliseconds / DebugLogInterval10Hz;
				double rms = sampleCountAcc > 0 ? Math.Sqrt(sumSqAcc / sampleCountAcc) : 0.0;
				Log($"GEN load: {msPerEpoch:F1} ms per 100 ms epoch ({msPerEpoch / 100.0 * 100.0:F0}% of real time) | SC08 rms={rms:F1} peak={peakAcc}/127");
				genStopwatch.Reset();
				LogTrackingDebug(grx, ieph, clipPct);
				clipCountAcc = 0;
				sampleCountAcc = 0;
				peakAcc = 0;
				sumSqAcc = 0.0;
			}

			// Every 30 seconds on GPS-time boundaries (match gps-sdr-sim).
			int igrx = (int)(grx.Sec * 10.0 + 0.5);
			if (igrx % 300 == 0)
			{
				for (int i = 0; i < MaxChan; i++)
				{
					if (_chan[i].Prn > 0)
						GenerateNavMsg(grx, _chan[i], false);
				}

				// Keep a single ephemeris slot for the entire run to maximize
				// receiver stability and avoid rapid in-view fluctuations caused by
				// runtime epoch switching. Re-enable epoch switching later if needed.

				xyz = (double[])CurrentXyz.Clone();
				AllocateChannels(grx, xyz, ieph);
			}

			grx = IncGpsTime(grx, 0.1);
		}

		Log("GPS signal engine stopped.");
	}

	// ── Hot sample-generation loop ──────────────────────────────────────────

	/// <summary>
	/// Flat, cache-friendly copy of the mutable per-channel state used by the
	/// inner loop. Avoids repeated class-field indirection per sample.
	/// </summary>
	private struct ActiveChan
	{
		public double CodePhase;
		public double FCode;
		public uint CarrPhase;
		public int CarrPhasestep;
		public int Gain;
		public int DataBit;
		public int CodeCA;
		public int Iword, Ibit, Icode;
	}

	private ActiveChan[] _active = new ActiveChan[MaxChan];
	private readonly int[] _activeIndex = new int[MaxChan];

	/// <summary>
	/// Generates one 0.1 s epoch of interleaved I/Q into <paramref name="iq_buff"/>.
	/// Only active channels are visited, state is held in a flat struct array, and
	/// all per-sample table/array accesses go through pinned pointers so the JIT
	/// emits no bounds checks in the innermost loop.
	/// </summary>
	private unsafe void GenerateEpoch(short[] iq_buff, int iq_buff_size, int[] chanGain, double delt)
	{
		// Compact active channels into a dense array.
		int nActive = 0;
		for (int i = 0; i < MaxChan; i++)
		{
			var ch = _chan[i];
			if (ch.Prn == 0) continue;
			_activeIndex[nActive] = i;
			_active[nActive] = new ActiveChan
			{
				CodePhase     = ch.CodePhase,
				FCode         = ch.FCode,
				CarrPhase     = ch.CarrPhase,
				CarrPhasestep = ch.CarrPhasestep,
				Gain          = chanGain[i],
				DataBit       = ch.DataBit,
				CodeCA        = ch.CodeCA,
				Iword         = ch.Iword,
				Ibit          = ch.Ibit,
				Icode         = ch.Icode,
			};
			nActive++;
		}

		fixed (ActiveChan* aBase = _active)
		fixed (short* iqBase = iq_buff)
		fixed (int* cosBase = CosTable512)
		fixed (int* sinBase = SinTable512)
		{
			for (int isamp = 0; isamp < iq_buff_size; isamp++)
			{
				int i_acc = 0, q_acc = 0;

				for (int a = 0; a < nActive; a++)
				{
					ActiveChan* c = aBase + a;
					var ch = _chan[_activeIndex[a]];

					int iTable = (int)((c->CarrPhase >> 16) & 0x1FFU); // top 9 bits → 0..511
					int common = c->DataBit * c->CodeCA * c->Gain;
					i_acc += cosBase[iTable] * common;
					q_acc += sinBase[iTable] * common;

					// Update code phase
					c->CodePhase += c->FCode * delt;
					if (c->CodePhase >= CaSeqLen)
					{
						c->CodePhase -= CaSeqLen;
						c->Icode++;

						if (c->Icode >= 20)
						{
							c->Icode = 0;
							c->Ibit++;

							if (c->Ibit >= 30)
							{
								c->Ibit = 0;
								if (++c->Iword >= ChannelState.NDwrd)
									c->Iword = 0;   // wrap nav message ring buffer (matches gpssim.c)
							}

							c->DataBit =
								(int)((ch.Dwrd[c->Iword] >> (29 - c->Ibit)) & 0x1UL) * 2 - 1;
						}
					}

					// Current C/A chip
					c->CodeCA = ch.Ca[(int)c->CodePhase] * 2 - 1;

					// Update carrier phase
					c->CarrPhase = (uint)(c->CarrPhase + (uint)c->CarrPhasestep);
				}

				// Scale by 2^7
				iqBase[isamp * 2]     = (short)((i_acc + 64) >> 7);
				iqBase[isamp * 2 + 1] = (short)((q_acc + 64) >> 7);
			}
		}

		// Write mutated state back to the canonical channel objects.
		for (int a = 0; a < nActive; a++)
		{
			var ch = _chan[_activeIndex[a]];
			ref var c = ref _active[a];
			ch.CodePhase = c.CodePhase;
			ch.CarrPhase = c.CarrPhase;
			ch.DataBit   = c.DataBit;
			ch.CodeCA    = c.CodeCA;
			ch.Iword     = c.Iword;
			ch.Ibit      = c.Ibit;
			ch.Icode     = c.Icode;
		}
	}

	// ── Channel allocation ───────────────────────────────────────────

	private static readonly bool[] AllocatedSat = new bool[MaxSat];

	private void AllocateChannels(GpsTime grx, double[] xyz, int ieph)
	{
		double[] llh = new double[3];
		Xyz2Llh(xyz, llh);

		double[][] tmat = MakeTmat();
		Ltcmat(llh, tmat);

		// Compute visibility for all SVs.
		// Two thresholds are used: _elevMask to acquire a new satellite, and a
		// slightly lower dropMask to keep one already being tracked (hysteresis).
		double dropMask = _elevMask - ElevMaskHysteresisDeg * Pi / 180.0;

		var visibleSvs = new List<(int sv, RangeData rho, double elev)>();
		var retainableBySv = new Dictionary<int, RangeData>(MaxSat);
		for (int sv = 0; sv < MaxSat; sv++)
		{
			if (!_eph[ieph][sv].Valid) continue;

			double[] pos = new double[3], vel = new double[3], clk = new double[2];
			SatPos(_eph[ieph][sv], grx, pos, vel, clk);

			double[] los = new double[3];
			SubVect(los, pos, xyz);
			double tau = NormVect(los) / SpeedOfLight;
			pos[0] -= vel[0] * tau; pos[1] -= vel[1] * tau; pos[2] -= vel[2] * tau;
			double xrot = pos[0] + pos[1] * OmegaEarth * tau;
			double yrot = pos[1] - pos[0] * OmegaEarth * tau;
			pos[0] = xrot; pos[1] = yrot;
			SubVect(los, pos, xyz);

			double[] neu = new double[3];
			Ecef2Neu(los, tmat, neu);
			double[] azel = new double[2];
			Neu2Azel(azel, neu);

			if (azel[1] > dropMask)
			{
				var rho = BuildRange(_eph[ieph][sv], grx, xyz, llh, tmat);
				// Above the drop threshold: eligible to be RETAINED.
				retainableBySv[sv] = rho;
				// Above the full mask: also eligible to be newly ACQUIRED.
				if (azel[1] > _elevMask)
					visibleSvs.Add((sv, rho, azel[1]));
			}
		}

		// Prefer highest-elevation satellites when adding new channels
		visibleSvs.Sort((a, b) => b.elev.CompareTo(a.elev));

		bool[] assigned = new bool[MaxSat];
		int nAssigned = 0;

		// 1) Keep existing channels if their SV is still above the DROP threshold
		//    (sticky allocation with hysteresis).
		for (int i = 0; i < MaxChan; i++)
		{
			int prn = _chan[i].Prn;
			if (prn <= 0) continue;

			int sv = prn - 1;
			if (sv < 0 || sv >= MaxSat || !retainableBySv.TryGetValue(sv, out var rho)
				|| nAssigned >= _maxActiveChan)
			{
				// Satellite no longer visible, or channel budget exhausted: free channel.
				_chan[i].Prn = 0;
				continue;
			}

			// Keep current channel state; only refresh az/el.
			_chan[i].Azel[0] = rho.Azel[0];
			_chan[i].Azel[1] = rho.Azel[1];
			assigned[sv] = true;
			nAssigned++;
		}

		// 2) Fill empty channels with best unassigned visible SVs.
		int next = 0;
		for (int i = 0; i < MaxChan; i++)
		{
			if (_chan[i].Prn > 0) continue;
			if (nAssigned >= _maxActiveChan) break;

			while (next < visibleSvs.Count && assigned[visibleSvs[next].sv])
				next++;

			if (next >= visibleSvs.Count)
				break;

			var (sv, rho, _) = visibleSvs[next++];
			int newPrn = sv + 1;

			_chan[i].Prn = newPrn;
			_chan[i].Azel[0] = rho.Azel[0];
			_chan[i].Azel[1] = rho.Azel[1];

			Codegen(_chan[i].Ca, newPrn);
			Eph2Sbf(_eph[ieph][sv], _ionoutc, _chan[i].Sbf);
			GenerateNavMsg(grx, _chan[i], true);

			_chan[i].Rho0 = rho;
			var rho1 = BuildRange(_eph[ieph][sv], IncGpsTime(grx, 0.1), xyz, llh, tmat);
			ComputeCodePhase(_chan[i], rho1, 0.1);

			// ComputeCodePhase leaves Rho0 = rho1 (the range at grx+0.1), but the
			// next UpdateActiveChannelCodePhase also runs at grx+0.1, which would
			// give rhorate = 0 and hence zero Doppler for one epoch on every newly
			// acquired satellite. Rewind the reference to the range at grx so the
			// first tracked epoch already carries the correct Doppler.
			_chan[i].Rho0 = rho;

			assigned[sv] = true;
			nAssigned++;
		}
	}

	private void ComputeCodePhase(ChannelState chan, RangeData rho1, double dt)
	{
		double rhorate = (rho1.Range - chan.Rho0.Range) / dt;

		chan.FCcarr = -rhorate / LambdaL1;
		chan.FCode  = CodeFreq + chan.FCcarr * CarrToCode;

		double norm_freq = chan.FCcarr / _sampFreq;
		chan.CarrPhasestep = (int)(norm_freq * 512.0 * 65536.0);

		double ms = ((SubGpsTime(chan.Rho0.G, chan.G0) + 6.0) - chan.Rho0.Range / SpeedOfLight) * 1000.0;
		int ims = (int)ms;
		chan.CodePhase = (ms - ims) * CaSeqLen;

		chan.Iword = ims / 600;
		if (chan.Iword >= ChannelState.NDwrd) chan.Iword %= ChannelState.NDwrd;
		ims -= chan.Iword * 600;
		chan.Ibit = ims / 20;
		ims -= chan.Ibit * 20;
		chan.Icode = ims;

		chan.CodeCA  = chan.Ca[(int)chan.CodePhase] * 2 - 1;
		chan.DataBit = (int)((chan.Dwrd[chan.Iword] >> (29 - chan.Ibit)) & 0x1UL) * 2 - 1;

		// Save current pseudorange as next epoch reference.
		chan.Rho0 = rho1;
	}

	private void UpdateActiveChannelCodePhase(GpsTime grx, double[] xyz, int ieph)
	{
		double[] llh = new double[3];
		Xyz2Llh(xyz, llh);
		double[][] tmat = MakeTmat();
		Ltcmat(llh, tmat);

		for (int i = 0; i < MaxChan; i++)
		{
			int prn = _chan[i].Prn;
			if (prn <= 0) continue;

			int sv = prn - 1;
			if (sv < 0 || sv >= MaxSat || !_eph[ieph][sv].Valid) continue;

			var rho = BuildRange(_eph[ieph][sv], grx, xyz, llh, tmat);
			_chan[i].Azel[0] = rho.Azel[0];
			_chan[i].Azel[1] = rho.Azel[1];
			ComputeCodePhase(_chan[i], rho, 0.1);
		}
	}

	private int FindBestEphemerisIndex(GpsTime grx)
	{
		int bestIdx = 0;
		double bestAbsDt = double.MaxValue;

		for (int i = 0; i < _eph.Length; i++)
		{
			double minAbsDtInSlot = double.MaxValue;
			bool anyValid = false;

			for (int sv = 0; sv < MaxSat; sv++)
			{
				if (!_eph[i][sv].Valid) continue;
				anyValid = true;
				double dt = Math.Abs(SubGpsTime(_eph[i][sv].Toc, grx));
				if (dt < minAbsDtInSlot) minAbsDtInSlot = dt;
			}

			if (anyValid && minAbsDtInSlot < bestAbsDt)
			{
				bestAbsDt = minAbsDtInSlot;
				bestIdx = i;
			}
		}

		return bestIdx;
	}

	private void LogTrackingDebug(GpsTime grx, int ieph, double clipPct)
	{
		var active = new List<ChannelState>(MaxChan);
		for (int i = 0; i < MaxChan; i++)
			if (_chan[i].Prn > 0)
				active.Add(_chan[i]);

		if (active.Count == 0)
		{
			Log($"DBG grx={grx.Week}:{grx.Sec:F1} ieph={ieph} active=0");
			return;
		}

		var c = active[0];
		int sfStart = (c.Iword / 10) * 10;
		int howIdx = Math.Clamp(sfStart + 1, 0, ChannelState.NDwrd - 1);
		ulong how = c.Dwrd[howIdx];
		ulong tow = (how >> 13) & 0x1FFFFUL;

		string sky = string.Join(", ",
			active.Take(Math.Min(4, active.Count)).Select(ch =>
				$"PRN{ch.Prn:D2}@{ch.Azel[0] * R2D:F0}/{ch.Azel[1] * R2D:F0}"));

		// Quick nav integrity probe for this PRN: expected subframe IDs over 6-word ring
		// are typically [5,1,2,3,4,5] at indices 1,11,21,31,41,51.
		int[] idIdx = { 1, 11, 21, 31, 41, 51 };
		string sfIds = string.Join("/", idIdx.Select(i => ((c.Dwrd[i] >> 8) & 0x7UL).ToString()));

		Log($"DBG grx={grx.Week}:{grx.Sec:F1} ieph={ieph} active={active.Count} " +
			$"PRN{c.Prn:D2} iword={c.Iword} ibit={c.Ibit} icode={c.Icode} " +
			$"tow={tow} sf={sfIds} clip={clipPct:F2}% g0={c.G0.Week}:{c.G0.Sec:F1} fCarr={c.FCcarr:F1} fCode={c.FCode:F1} " +
			$"azel={c.Azel[0] * R2D:F1}/{c.Azel[1] * R2D:F1} | sky={sky}");
	}

	// ── Range computation ────────────────────────────────────────────

	private RangeData BuildRange(Ephemeris eph, GpsTime g, double[] xyz, double[] llh, double[][] tmat)
	{
		double[] pos = new double[3], vel = new double[3], clk = new double[2];
		SatPos(eph, g, pos, vel, clk);

		double[] los = new double[3];
		SubVect(los, pos, xyz);
		double tau = NormVect(los) / SpeedOfLight;

		pos[0] -= vel[0] * tau; pos[1] -= vel[1] * tau; pos[2] -= vel[2] * tau;
		double xrot = pos[0] + pos[1] * OmegaEarth * tau;
		double yrot = pos[1] - pos[0] * OmegaEarth * tau;
		pos[0] = xrot; pos[1] = yrot;

		SubVect(los, pos, xyz);
		double range = NormVect(los);

		double[] neu = new double[3];
		Ecef2Neu(los, tmat, neu);
		double[] azel = new double[2];
		Neu2Azel(azel, neu);

		double iDelay = IonoDelay(_ionoutc, g, llh, azel);

		double rate = DotProd(vel, los) / range;

		return new RangeData
		{
			G         = g,
			D         = range,
			Range     = range - SpeedOfLight * clk[0] + iDelay,
			Rate      = rate,
			Azel      = new double[] { azel[0], azel[1] },
			IonoDelay = iDelay
		};
	}

	private void Log(string msg) => LogMessage?.Invoke($"[{DateTime.Now:HH:mm:ss}] {msg}");
}
