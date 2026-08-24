using System.Runtime.Versioning;
using GPSSimulator.GpsEngine;
using static GPSSimulator.GpsEngine.GpsMath;

namespace GPSSimulator.Services;

/// <summary>
/// Orchestrates GPS L1 C/A signal generation (via GpsSignalEngine) and
/// streaming to HackRF One. Supports live lat/lon updates while transmitting.
/// </summary>
public class GpsSimulatorService
{
	public event Action<string>? LogMessage;
	public bool IsRunning { get; private set; }

	private CancellationTokenSource? _cts;
	private GpsSignalEngine? _engine;

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

		// Find the start GPS time from the first valid ephemeris
		GpsTime g0 = GpsTime.Zero;
		for (int sv = 0; sv < GpsConstants.MaxSat; sv++)
		{
			if (eph[0][sv].Valid) { g0 = eph[0][sv].Toc; break; }
		}
		Log($"Scenario start: GPS week {g0.Week}, sec {g0.Sec:F0}");

		_engine = new GpsSignalEngine(eph, ionoutc, settings.SampleRateMHz * 1e6, settings.ElevMaskDeg);
		_engine.LogMessage += Log;

		// Set initial ECEF position
		UpdatePosition(settings.Latitude, settings.Longitude, settings.AltitudeMeters);

#if WINDOWS
		await TransmitOnWindowsAsync(settings, g0, ct);
#else
		Log("HackRF transmission is only supported on Windows.");
		await Task.CompletedTask;
#endif
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
			device.FilterBandwidthMHz  = settings.SampleRateMHz;
			device.TXVGAGainDb         = settings.TxGainDb;
			device.AMPEnable           = settings.AmpEnabled;

			Log($"HackRF configured: 1575.42 MHz | {settings.SampleRateMHz} MHz SR | {settings.TxGainDb} dB VGA");

			txStream = device.StartTX();
			Log("Streaming live GPS IQ → HackRF (update lat/lon anytime)...");

			using var limitCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
			if (settings.DurationSeconds > 0)
				limitCts.CancelAfter(TimeSpan.FromSeconds(settings.DurationSeconds));

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

public class SimulatorSettings
{
	public string RinexNavFilePath { get; set; } = string.Empty;
	public double Latitude         { get; set; }
	public double Longitude        { get; set; }
	public double AltitudeMeters   { get; set; } = 100.0;
	public int    DurationSeconds  { get; set; } = 0;       // 0 = run until stopped
	public double SampleRateMHz    { get; set; } = 2.6;
	public int    TxGainDb         { get; set; } = 20;
	public bool   AmpEnabled       { get; set; } = false;
	public double ElevMaskDeg      { get; set; } = 0.0;
}
