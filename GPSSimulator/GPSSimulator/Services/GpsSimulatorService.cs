using System.Runtime.Versioning;
using GPSSimulator.GpsEngine;
using static GPSSimulator.GpsEngine.GpsMath;

namespace GPSSimulator.Services;

/// <summary>Fired each time the trip replay advances to a new point.</summary>
public record TripProgressEvent(
    int      PointIndex,
    int      PointCount,
    double   Latitude,
    double   Longitude,
    double   AltitudeMeters,
    double   SpeedKph,
    int      HeadingDeg,
    TimeSpan Elapsed,
    TimeSpan Total,
    DateTime SimulatedUtc   // original timestamp from the trip data
);

/// <summary>
/// Orchestrates GPS L1 C/A signal generation (via GpsSignalEngine) and
/// streaming to HackRF One. Supports live lat/lon updates while transmitting.
/// </summary>
public class GpsSimulatorService
{
	public event Action<string>? LogMessage;
	public event Action<TripProgressEvent>? TripProgress;
	public bool IsRunning { get; private set; }

	private CancellationTokenSource? _cts;
	private GpsSignalEngine? _engine;

	// ── Trip replay ───────────────────────────────────────────────────────────

	/// <summary>
	/// Start the simulator with an AxonTrip replay.
	/// Timestamps in the trip are rebased so the trip begins at the RINEX
	/// simulation start time.  Each position is pushed to the IQ engine at
	/// the correct wall-clock interval so replay runs at 1× real speed.
	/// </summary>
	public async Task StartTripReplayAsync(SimulatorSettings settings, AxonTrip trip)
	{
		if (IsRunning) throw new InvalidOperationException("Already running.");
		ValidateSettings(settings);

		Log($"Trip replay loaded: {trip.Summary}");

		_cts    = new CancellationTokenSource();
		IsRunning = true;
		try
		{
			// Use first trip point's lat/lon/alt as the simulation start position
			var first = trip.Points[0];
			settings = settings with
			{
				Latitude       = first.Latitude,
				Longitude      = first.Longitude,
				AltitudeMeters = first.AltitudeMeters,
			};

			// Build engine with trip initial position
			var g0 = await PrepareEngineAsync(settings, _cts.Token);

					// Run IQ stream and trip position driver in parallel.
					// Use a linked CTS so either task can abort the other on failure.
					using var linkedCts = CancellationTokenSource.CreateLinkedTokenSource(_cts.Token);

					var simStartUtc = GpsMath.GpsTimeToDateTimeUtc(g0);
					var driverTask = RunTripDriverAsync(trip, simStartUtc, linkedCts.Token);

			#if WINDOWS
					var streamTask = TransmitOnWindowsAsync(settings, g0, linkedCts.Token);
			#else
					Log("HackRF transmission is only supported on Windows.");
					var streamTask = Task.CompletedTask;
			#endif
					try
					{
						await Task.WhenAll(driverTask, streamTask);
					}
					catch
					{
						// Cancel the other task if one faults, then re-throw the first real error
						linkedCts.Cancel();
						await Task.WhenAll(
							driverTask.ContinueWith(_ => { }, TaskContinuationOptions.None),
							streamTask.ContinueWith(_ => { }, TaskContinuationOptions.None));

						// Propagate the first non-cancellation exception
						var ex = (driverTask.Exception ?? streamTask.Exception)?.InnerException;
						if (ex != null && ex is not OperationCanceledException)
							throw ex;
					}
		}
		finally { IsRunning = false; _engine = null; }
	}

	/// <summary>
	/// Drive position updates from the trip points at 1× real-time speed.
	/// Rebases trip timestamps so they are relative to wall-clock start.
	/// </summary>
	private async Task RunTripDriverAsync(AxonTrip trip, DateTime simStartUtc, CancellationToken ct)
	{
		var points    = trip.Points;
		var wallStart = DateTime.UtcNow;
		var total     = trip.TotalDuration;

		Log($"Trip driver: {points.Count} points, duration {total:hh\\:mm\\:ss}");

		if (points.Count == 0) { Log("Trip has no points."); return; }

		// Drive the position at the same 10 Hz rate the signal engine samples it.
		// Stepping only once per trip point leaves CurrentXyz frozen for most
		// epochs, so the pseudorange rate (and therefore the Doppler the receiver
		// sees) carries no user velocity, and the receiver reports 0 km/h even
		// though the position keeps jumping forward.
		const double TickSeconds = 0.1;

		double endSeconds  = points[^1].OffsetSeconds;
		int    seg         = 0;   // index of the trip point at or before "now"
		int    lastReported = -1;

		while (!ct.IsCancellationRequested)
		{
			double t = (DateTime.UtcNow - wallStart).TotalSeconds;
			if (t > endSeconds) break;

			// Advance to the segment bracketing the current replay time
			while (seg < points.Count - 2 && points[seg + 1].OffsetSeconds <= t)
				seg++;

			var a = points[seg];
			var b = points[Math.Min(seg + 1, points.Count - 1)];

			double span = b.OffsetSeconds - a.OffsetSeconds;
			double f    = span > 1e-9 ? Math.Clamp((t - a.OffsetSeconds) / span, 0.0, 1.0) : 0.0;

			double lat = a.Latitude       + (b.Latitude       - a.Latitude)       * f;
			double lon = a.Longitude      + (b.Longitude      - a.Longitude)      * f;
			double alt = a.AltitudeMeters + (b.AltitudeMeters - a.AltitudeMeters) * f;

			UpdatePosition(lat, lon, alt);

			if (seg != lastReported)
			{
				lastReported = seg;
				var targetElapsed = TimeSpan.FromSeconds(a.OffsetSeconds);

				TripProgress?.Invoke(new TripProgressEvent(
					PointIndex    : seg,
					PointCount    : points.Count,
					Latitude      : lat,
					Longitude     : lon,
					AltitudeMeters: alt,
					SpeedKph      : a.SpeedKph,
					HeadingDeg    : a.HeadingDeg,
					Elapsed       : targetElapsed,
					Total         : total,
					SimulatedUtc  : simStartUtc + targetElapsed
				));

				// Log every 60 seconds of trip time
				if (seg % 60 == 0)
					Log($"[Trip {seg + 1}/{points.Count}] {lat:F6}, {lon:F6} | " +
						$"{a.SpeedKph:F0} km/h | {targetElapsed:hh\\:mm\\:ss} elapsed");
			}

			try { await Task.Delay(TimeSpan.FromSeconds(TickSeconds), ct); }
			catch (OperationCanceledException) { break; }
		}

		Log("Trip replay finished.");
		_cts?.Cancel(); // stop the IQ stream when the trip ends
	}

	/// <summary>
	/// Update the simulated receiver position while the engine is running.
	/// Safe to call from any thread at any time.
	/// </summary>
	public void UpdatePosition(double latDeg, double lonDeg, double altMetres)
	{
		if (_engine == null) return;
		double[] llh = { latDeg * Math.PI / 180.0, lonDeg * Math.PI / 180.0, altMetres };
		double[] xyz = new double[3];
		Llh2Xyz(llh, xyz);
		_engine.CurrentXyz = xyz;
	}

	public async Task StartAsync(SimulatorSettings settings)
	{
		if (IsRunning) throw new InvalidOperationException("Already running.");
		ValidateSettings(settings);

		_cts = new CancellationTokenSource();
		IsRunning = true;
		try { await RunAsync(settings, _cts.Token); }
		finally { IsRunning = false; _engine = null; }
	}

	public void Stop() => _cts?.Cancel();

	// ── Core pipeline ────────────────────────────────────────────────

	private async Task RunAsync(SimulatorSettings settings, CancellationToken ct)
	{
		var g0 = await PrepareEngineAsync(settings, ct);
#if WINDOWS
		await TransmitOnWindowsAsync(settings, g0, ct);
#else
		Log("HackRF transmission is only supported on Windows.");
		await Task.CompletedTask;
#endif
	}

	/// <summary>
	/// Shared engine setup used by both static and trip-replay modes.
	/// Loads RINEX, creates GpsSignalEngine, sets initial position.
	/// Returns the simulation start GPS time (g0).
	/// </summary>
	private async Task<GpsTime> PrepareEngineAsync(SimulatorSettings settings, CancellationToken ct)
	{
		Log("Loading RINEX navigation file...");

		var eph = new Ephemeris[GpsConstants.EphemArraySize][];
		for (int i = 0; i < GpsConstants.EphemArraySize; i++)
		{
			eph[i] = new Ephemeris[GpsConstants.MaxSat];
			for (int sv = 0; sv < GpsConstants.MaxSat; sv++)
				eph[i][sv] = new Ephemeris();
		}

		var ionoutc = new IonoUtc();
		int neph = GpsRinexParser.ReadRinexNavAll(eph, ionoutc, settings.RinexNavFilePath);

		if (neph <= 0)
			throw new Exception("Failed to read RINEX file. Ensure it is a valid RINEX 2.11 GPS nav file.");

		Log($"Loaded {neph} ephemeris epoch(s). Ionosphere model: {(ionoutc.Vflg ? "Klobuchar" : "default 5 ns")}");

		GpsTime g0 = GpsTime.Zero;
		for (int sv = 0; sv < GpsConstants.MaxSat; sv++)
		{
			if (eph[0][sv].Valid) { g0 = eph[0][sv].Toc; break; }
		}

		// If "use current time" is requested, override g0 with DateTime.UtcNow
		// and verify the RINEX file actually covers this moment.
		if (settings.UseCurrentTime)
		{
			var now    = DateTime.UtcNow;
			var nowGps = DateTimeUtcToGpsTime(now);

			// Gather the UTC coverage window (all valid Toc ± 2 h)
			var times = new List<DateTime>();
			for (int i = 0; i < GpsConstants.EphemArraySize; i++)
				for (int sv = 0; sv < GpsConstants.MaxSat; sv++)
					if (eph[i][sv].Valid)
						times.Add(GpsTimeToDateTimeUtc(eph[i][sv].Toc));

			var coverageStart = times.Min().AddHours(-2);
			var coverageEnd   = times.Max().AddHours(+2);

			if (now < coverageStart || now > coverageEnd)
				throw new InvalidOperationException(
					$"\"Use current time\" is enabled, but the loaded RINEX file does not cover now " +
					$"({now:yyyy-MM-dd HH:mm}Z). " +
					$"File covers {coverageStart:yyyy-MM-dd HH:mm}Z – {coverageEnd:yyyy-MM-dd HH:mm}Z. " +
					$"Download a current RINEX broadcast file (e.g. brdc{now:DDD}.{now:yy}n from NASA CDDIS).");

			g0 = nowGps;
			Log($"Scenario start overridden to current time: {now:yyyy-MM-dd HH:mm:ss}Z " +
				$"(GPS week {g0.Week}, sec {g0.Sec:F0})");
		}
		else
		{
			Log($"Scenario start: GPS week {g0.Week}, sec {g0.Sec:F0} " +
				$"({GpsTimeToDateTimeUtc(g0):yyyy-MM-dd HH:mm:ss}Z)");
		}

		_engine = new GpsSignalEngine(eph, ionoutc, settings.SampleRateMHz * 1e6, settings.ElevMaskDeg, settings.MaxSatellites);
		_engine.LogMessage += Log;

		UpdatePosition(settings.Latitude, settings.Longitude, settings.AltitudeMeters);

		_ = ct; // suppress unused warning; kept for signature parity
		return g0;
	}

#if WINDOWS
	[SupportedOSPlatform("windows")]
	private async Task TransmitOnWindowsAsync(SimulatorSettings settings, GpsTime g0, CancellationToken ct)
	{
		// Guard: attempt to load hackrf.dll BEFORE touching the NetHackrf type.
		// NetHackrf has an eager static constructor that calls hackrf_init() — if the DLL
		// cannot be loaded (missing or wrong bitness), it poisons the type and causes
		// cascading TypeInitializationExceptions. We detect the failure here first.
		if (!System.Runtime.InteropServices.NativeLibrary.TryLoad("hackrf.dll", out IntPtr _))
		{
			throw new Exception(
				"hackrf.dll could not be loaded. " +
				"Ensure a 64-bit (x64) hackrf.dll is present in the application directory. " +
				"Download the Windows x64 build from https://github.com/greatscottgadgets/hackrf/releases " +
				"and copy hackrf.dll next to GPSSimulator.exe.");
		}

		nethackrf.NetHackrf? device = null;
		System.IO.Stream? txStream = null;

		try
		{
			nethackrf.NetHackrf.hackrf_device_info[] devices;
			try
			{
				devices = nethackrf.NetHackrf.HackrfDeviceList();
			}
			catch (TypeInitializationException ex)
			{
				throw new Exception(
					$"HackRF library initialization failed: {ex.InnerException?.Message ?? ex.Message}. " +
					"Verify that a compatible 64-bit hackrf.dll is present.", ex);
			}

			if (devices.Length == 0)
				throw new Exception("No HackRF device found. Check USB connection and drivers.");

			Log($"Opening HackRF ({devices[0].serial_number})...");
			device = devices[0].OpenDevice();

			device.CarrierFrequencyMHz = 1575.42;
			device.SampleFrequencyMHz  = settings.SampleRateMHz;
			// Baseband filter must be wider than the sample rate, otherwise the
			// C/A main lobe (+/-1.023 MHz) is clipped and C/N0 collapses.
			// multi-sdr-gps-sim uses 2x the sample rate.
			device.FilterBandwidthMHz  = settings.SampleRateMHz * 2.0;
			device.TXVGAGainDb         = settings.TxGainDb;
			device.AMPEnable           = settings.AmpEnabled;
			device.AntPower            = false; // bias tee off

			Log($"HackRF configured: 1575.42 MHz | {settings.SampleRateMHz} MHz SR | {settings.SampleRateMHz * 2.0:F1} MHz BW | {settings.TxGainDb} dB VGA");

			var hackrfTx = device.StartTX();
			txStream = hackrfTx;
			Log("Streaming live GPS IQ → HackRF (update lat/lon anytime)...");

			using var limitCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
			if (settings.DurationSeconds > 0)
				limitCts.CancelAfter(TimeSpan.FromSeconds(settings.DurationSeconds));

			// Periodically report TX underruns so streaming continuity is visible
			// in the log without attaching a debugger.
			var underrunMonitor = Task.Run(async () =>
			{
				long last = 0;
				try
				{
					while (!limitCts.Token.IsCancellationRequested)
					{
						await Task.Delay(10000, limitCts.Token);
						long now = hackrfTx.TxUnderrunCount;
						if (now != last)
						{
							Log($"TX underruns: {now} (+{now - last} in last 10 s)");
							last = now;
						}
					}
				}
				catch (OperationCanceledException) { }
			}, limitCts.Token);

			await _engine!.StreamAsync(txStream, g0, limitCts.Token);
		}
		catch (OperationCanceledException) { Log("Transmission stopped."); }
		finally
		{
			txStream?.Dispose();
			device?.Dispose();
			Log("HackRF disconnected.");
		}
	}
#endif

	/// <summary>
	/// Reads the RINEX file header and ephemeris epochs to determine the UTC time
	/// window the file covers. Returns a <see cref="RinexCoverageResult"/> that
	/// the UI can use to warn the user when "Use current time" is selected.
	/// </summary>
	public RinexCoverageResult CheckRinexCoverage(string rinexPath)
	{
		if (!File.Exists(rinexPath))
			return RinexCoverageResult.FileNotFound;

		try
		{
			var eph = new Ephemeris[GpsConstants.EphemArraySize][];
			for (int i = 0; i < GpsConstants.EphemArraySize; i++)
			{
				eph[i] = new Ephemeris[GpsConstants.MaxSat];
				for (int sv = 0; sv < GpsConstants.MaxSat; sv++)
					eph[i][sv] = new Ephemeris();
			}
			var ionoutc = new IonoUtc();
			int neph = GpsRinexParser.ReadRinexNavAll(eph, ionoutc, rinexPath);
			if (neph <= 0) return RinexCoverageResult.ParseError;

			// Collect all valid Toc values and convert to UTC
			var times = new List<DateTime>();
			for (int i = 0; i < GpsConstants.EphemArraySize; i++)
				for (int sv = 0; sv < GpsConstants.MaxSat; sv++)
					if (eph[i][sv].Valid)
						times.Add(GpsTimeToDateTimeUtc(eph[i][sv].Toc));

			if (times.Count == 0) return RinexCoverageResult.ParseError;

			// Each ephemeris is valid ±2 h around its Toc
			var coverageStart = times.Min().AddHours(-2);
			var coverageEnd   = times.Max().AddHours(+2);
			var now           = DateTime.UtcNow;
			bool coversNow    = now >= coverageStart && now <= coverageEnd;

			return new RinexCoverageResult(
				IsValid      : true,
				CoverageStart: coverageStart,
				CoverageEnd  : coverageEnd,
				CoversNow    : coversNow,
				Error        : null);
		}
		catch (Exception ex)
		{
			return new RinexCoverageResult(false, DateTime.MinValue, DateTime.MinValue, false, ex.Message);
		}
	}

	private void ValidateSettings(SimulatorSettings s)
	{
		if (!File.Exists(s.RinexNavFilePath))
			throw new FileNotFoundException($"RINEX nav file not found: {s.RinexNavFilePath}");
		if (s.Latitude < -90 || s.Latitude > 90)
			throw new ArgumentOutOfRangeException(nameof(s.Latitude), "Must be -90..90.");
		if (s.Longitude < -180 || s.Longitude > 180)
			throw new ArgumentOutOfRangeException(nameof(s.Longitude), "Must be -180..180.");
	}

	private void Log(string message) =>
		LogMessage?.Invoke($"[{DateTime.Now:HH:mm:ss}] {message}");
}

public record SimulatorSettings
{
	public string RinexNavFilePath { get; set; } = string.Empty;
	public double Latitude         { get; set; }
	public double Longitude        { get; set; }
	public double AltitudeMeters   { get; set; } = 100.0;
	public int    DurationSeconds  { get; set; } = 0;       // 0 = run until stopped
	public double SampleRateMHz    { get; set; } = 3.0;
	public int    TxGainDb         { get; set; } = 30;
	public bool   AmpEnabled       { get; set; } = true;
	public double ElevMaskDeg      { get; set; } = 10.0;
	/// <summary>
	/// Number of satellites transmitted. A fix needs 4; using fewer channels
	/// gives each satellite a larger share of the SC08 dynamic range, which
	/// raises per-satellite C/N0 and makes acquisition far more reliable.
	/// </summary>
	public int    MaxSatellites    { get; set; } = 8;
	/// <summary>
	/// When true, the simulation's GPS start time is set to DateTime.UtcNow
	/// instead of the first epoch in the RINEX file.
	/// The RINEX file must cover the current time for this to succeed.
	/// </summary>
	public bool   UseCurrentTime   { get; set; } = false;
}

/// <summary>
/// Result of proactively checking whether a RINEX file covers a given time.
/// </summary>
public record RinexCoverageResult(
	bool     IsValid,
	DateTime CoverageStart,
	DateTime CoverageEnd,
	bool     CoversNow,
	string?  Error)
{
	public static readonly RinexCoverageResult FileNotFound =
		new(false, DateTime.MinValue, DateTime.MinValue, false, "File not found.");
	public static readonly RinexCoverageResult ParseError =
		new(false, DateTime.MinValue, DateTime.MinValue, false, "Could not parse RINEX file.");
	public static readonly RinexCoverageResult Unknown =
		new(false, DateTime.MinValue, DateTime.MinValue, false, null);
}
