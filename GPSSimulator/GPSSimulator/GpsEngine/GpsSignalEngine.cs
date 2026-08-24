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

	// ── private state ────────────────────────────────────────────────

	private double[] _xyz = new double[3];

	private readonly Ephemeris[][] _eph;
	private readonly IonoUtc _ionoutc;
	private readonly ChannelState[] _chan;
	private readonly double _sampFreq;
	private readonly double _elevMask;

	private const int MaxChan = GpsConstants.MaxChan;
	private const int MaxSat  = GpsConstants.MaxSat;

	public GpsSignalEngine(
		Ephemeris[][] eph,
		IonoUtc ionoutc,
		double sampFreq = DefaultSampleRate,
		double elevMaskDeg = 0.0)
	{
		_eph      = eph;
		_ionoutc  = ionoutc;
		_sampFreq = sampFreq;
		_elevMask = elevMaskDeg * Pi / 180.0;
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
		int ieph = 0;

		// Find a valid ephemeris set to start with
		for (int sv = 0; sv < MaxSat; sv++)
			if (_eph[0][sv].Valid) { break; }

		// Allocate initial channels
		double[] xyz0 = (double[])CurrentXyz.Clone();
		AllocateChannels(grx, xyz0, ieph);

		// IQ buffers
		short[] iq_buff   = new short[iq_buff_size * 2];
		byte[]  iq8_buff  = new byte [iq_buff_size * 2];

		Log($"GPS signal engine started. Sample rate: {samp_freq / 1e6:F2} MHz, buffer: {iq_buff_size} samples/epoch");

		while (!ct.IsCancellationRequested)
		{
			double[] xyz = (double[])CurrentXyz.Clone(); // snapshot position

			// ── Generate one 0.1-second epoch of IQ samples ────────────
			for (int isamp = 0; isamp < iq_buff_size; isamp++)
			{
				int i_acc = 0, q_acc = 0;

				for (int i = 0; i < MaxChan; i++)
				{
					if (_chan[i].Prn == 0) continue;

					// Lookup table index from carrier phase accumulator
					int iTable = (int)((_chan[i].CarrPhase >> 16) & 0x1FFU); // top 9 bits → 0..511

					// Antenna gain
					double azel_elev = _chan[i].Azel[1];
					int iang = (int)(90.0 - azel_elev * R2D);
					iang = Math.Clamp(iang / 5, 0, 36); // 0:5:180 => index 0-36
					double gain = antPat[iang];

					int igain = (int)(gain * 128.0);

					int ip = _chan[i].DataBit * _chan[i].CodeCA * CosTable512[iTable] * igain;
					int qp = _chan[i].DataBit * _chan[i].CodeCA * SinTable512[iTable] * igain;

					i_acc += ip;
					q_acc += qp;

					// Update code phase
					_chan[i].CodePhase += _chan[i].FCode * delt;
					if (_chan[i].CodePhase >= CaSeqLen)
					{
						_chan[i].CodePhase -= CaSeqLen;
						_chan[i].Icode++;

						if (_chan[i].Icode >= 20)
						{
							_chan[i].Icode = 0;
							_chan[i].Ibit++;

							if (_chan[i].Ibit >= 30)
							{
								_chan[i].Ibit = 0;
								_chan[i].Iword++;
							}

							_chan[i].DataBit =
								(int)((_chan[i].Dwrd[_chan[i].Iword] >> (29 - _chan[i].Ibit)) & 0x1UL) * 2 - 1;
						}
					}

					// Current C/A chip
					_chan[i].CodeCA = _chan[i].Ca[(int)_chan[i].CodePhase] * 2 - 1;

					// Update carrier phase
					_chan[i].CarrPhase = (uint)(_chan[i].CarrPhase + (uint)_chan[i].CarrPhasestep);
				}

				// Scale by 2^7
				i_acc = (i_acc + 64) >> 7;
				q_acc = (q_acc + 64) >> 7;

				iq_buff[isamp * 2]     = (short)i_acc;
				iq_buff[isamp * 2 + 1] = (short)q_acc;
			}

			// SC08: shift 12-bit bladeRF values to 8-bit HackRF
			for (int isamp = 0; isamp < iq_buff_size * 2; isamp++)
				iq8_buff[isamp] = (byte)(sbyte)(iq_buff[isamp] >> 4);

			await output.WriteAsync(iq8_buff, 0, iq8_buff.Length, ct);

			// ── Every 30 seconds: update nav messages + reallocate ─────
			int igrx = (int)(grx.Sec * 10.0 + 0.5);
			if (igrx % 300 == 0)
			{
				for (int i = 0; i < MaxChan; i++)
					if (_chan[i].Prn > 0)
						GenerateNavMsg(grx, _chan[i], false);

				// Refresh ephemeris if a newer set is available
				if (ieph + 1 < _eph.Length)
				{
					for (int sv = 0; sv < MaxSat; sv++)
					{
						if (_eph[ieph + 1][sv].Valid)
						{
							double dt = SubGpsTime(_eph[ieph + 1][sv].Toc, grx);
							if (dt < SecondsInHour)
							{
								ieph++;
								for (int i = 0; i < MaxChan; i++)
									if (_chan[i].Prn != 0)
										Eph2Sbf(_eph[ieph][_chan[i].Prn - 1], _ionoutc, _chan[i].Sbf);
								break;
							}
						}
					}
				}

				xyz = (double[])CurrentXyz.Clone();
				AllocateChannels(grx, xyz, ieph);
			}

			grx = IncGpsTime(grx, 0.1);
		}

		Log("GPS signal engine stopped.");
	}

	// ── Channel allocation ───────────────────────────────────────────

	private static readonly bool[] AllocatedSat = new bool[MaxSat];

	private void AllocateChannels(GpsTime grx, double[] xyz, int ieph)
	{
		double[] llh = new double[3];
		Xyz2Llh(xyz, llh);

		double[][] tmat = MakeTmat();
		Ltcmat(llh, tmat);

		// Mark which SVs are already tracked
		bool[] used = new bool[MaxSat];
		for (int i = 0; i < MaxChan; i++)
			if (_chan[i].Prn > 0) used[_chan[i].Prn - 1] = true;

		// Compute visibility for all SVs
		var visibleSvs = new List<(int sv, RangeData rho, double azel_elev)>();

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

			if (azel[1] > _elevMask)
			{
				var rho = BuildRange(_eph[ieph][sv], grx, xyz, llh, tmat);
				visibleSvs.Add((sv, rho, azel[1]));
			}
		}

		// Sort by elevation descending
		visibleSvs.Sort((a, b) => b.azel_elev.CompareTo(a.azel_elev));

		// Assign visible SVs to channels
		bool[] assigned = new bool[MaxSat];
		for (int i = 0; i < MaxChan; i++)
		{
			if (i < visibleSvs.Count)
			{
				var (sv, rho, _) = visibleSvs[i];
				int prn = sv + 1;

				if (_chan[i].Prn == prn) continue; // already tracking

				// New SV on this channel
				_chan[i].Prn = prn;
				_chan[i].Azel[0] = rho.Azel[0];
				_chan[i].Azel[1] = rho.Azel[1];

				// Generate C/A code
				Codegen(_chan[i].Ca, prn);

				// Encode subframes
				Eph2Sbf(_eph[ieph][sv], _ionoutc, _chan[i].Sbf);

				// Generate navigation message
				GenerateNavMsg(grx, _chan[i], true);

				// Compute initial code/carrier phase
				var rho1 = BuildRange(_eph[ieph][sv], IncGpsTime(grx, 0.1), xyz, llh, tmat);
				ComputeCodePhase(_chan[i], rho, rho1, 0.1);

				assigned[sv] = true;
			}
			else
			{
				_chan[i].Prn = 0;
			}
		}
	}

	private void ComputeCodePhase(ChannelState chan, RangeData rho0, RangeData rho1, double dt)
	{
		double rhorate = (rho1.Range - rho0.Range) / dt;

		chan.FCcarr  = -rhorate / LambdaL1;
		chan.FCode   = CodeFreq + chan.FCcarr * CarrToCode;

		// Carrier phase step: (f_carr / samp_freq) * 512 table entries, scaled to uint32
		// We use 32-bit accumulator, 512-entry table. Phase step per sample:
		//   step = (f_carr / samp_freq) * 512  → multiply by 2^23 for fixed-point
		double norm_freq = chan.FCcarr / _sampFreq;   // -0.5 .. +0.5
		chan.CarrPhasestep = (int)(norm_freq * 512.0 * 65536.0); // 16.16 fixed-point into 9-bit table

		double ms = ((SubGpsTime(rho0.G, chan.G0) + 6.0) - rho0.Range / SpeedOfLight) * 1000.0;
		int ims = (int)ms;
		chan.CodePhase = (ms - ims) * CaSeqLen;

		chan.Iword = ims / 600;
		ims -= chan.Iword * 600;
		chan.Ibit  = ims / 20;
		ims -= chan.Ibit * 20;
		chan.Icode = ims;

		chan.CodeCA  = chan.Ca[(int)chan.CodePhase] * 2 - 1;
		chan.DataBit = (int)((chan.Dwrd[chan.Iword] >> (29 - chan.Ibit)) & 0x1UL) * 2 - 1;

		chan.Rho0 = rho0;
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
