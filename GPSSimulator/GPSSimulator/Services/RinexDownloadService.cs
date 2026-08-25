using System.Diagnostics;
using System.IO.Compression;

namespace GPSSimulator.Services;

/// <summary>
/// Downloads the latest IGS broadcast RINEX 2.11 GPS nav file from NASA CDDIS FTPS.
/// Uses Windows built-in curl.exe for explicit TLS FTPS (port 21, anonymous login).
///
/// Two products are used, in order:
///  1. HOURLY: /pub/gps/data/hourly/{year}/{doy}/hour{doy}0.{yy}n.gz
///     Updated through the day and typically only 15-60 min behind real time.
///     This is what you want when simulating "now".
///  2. DAILY:  /pub/gps/data/daily/{year}/brdc/brdc{doy}0.{yy}n.gz
///     The current day's daily file is built up incrementally and can lag real
///     time by several hours, so it is only used as a fallback.
/// </summary>
public class RinexDownloadService
{
	private const string FtpHost = "gdc.cddis.eosdis.nasa.gov";
	private const string DailyPathFmt  = "/pub/gps/data/daily/{0}/brdc/brdc{1}0.{2}n.gz";
	private const string HourlyPathFmt = "/pub/gps/data/hourly/{0}/{1}/hour{1}0.{2}n.gz";

	/// <summary>How many hourly snapshots to walk back through before giving up.</summary>
	private const int HourlyLookbackHours = 6;

	private static readonly string CurlExe =
		Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.System), "curl.exe");

	public static string DownloadFolder =>
		Path.Combine(
			Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
			"GPSSimulator", "rinex");

	public record DownloadResult(bool Success, string? FilePath, string Message);

	public async Task<DownloadResult> DownloadLatestAsync(
		IProgress<string> progress,
		CancellationToken ct = default)
	{
		if (!File.Exists(CurlExe))
			return new DownloadResult(false, null,
				$"curl.exe not found at {CurlExe}. Windows 10 1803+ required.");

		Directory.CreateDirectory(DownloadFolder);

		// ── 1) Hourly files: freshest source, best for simulating current time ──
		var nowUtc = DateTime.UtcNow;
		for (int hoursBack = 0; hoursBack <= HourlyLookbackHours; hoursBack++)
		{
			if (ct.IsCancellationRequested)
				return new DownloadResult(false, null, "Download cancelled.");

			var stamp   = nowUtc.AddHours(-hoursBack);
			int doy     = stamp.DayOfYear;
			string yy   = stamp.ToString("yy");
			string doyS = doy.ToString("D3");

			// Cache per acquisition hour so a stale snapshot is never reused.
			string fileName = $"hour{doyS}0.{yy}n";
			string cacheName = $"{fileName}.{stamp:HH}";
			string gzPath   = Path.Combine(DownloadFolder, cacheName + ".gz");
			string navPath  = Path.Combine(DownloadFolder, cacheName);
			string ftpUrl   = $"ftp://{FtpHost}" + string.Format(HourlyPathFmt, stamp.Year, doyS, yy);

			if (File.Exists(navPath))
			{
				progress.Report($"Using cached hourly file: {navPath}");
				return new DownloadResult(true, navPath, $"Loaded from cache: {cacheName}");
			}

			progress.Report(hoursBack == 0
				? $"Trying hourly file ({stamp:yyyy-MM-dd HH}:00Z, DOY {doy})..."
				: $"Trying hourly file {hoursBack} h back ({stamp:yyyy-MM-dd HH}:00Z)...");

			var hourly = await TryFetchAsync(ftpUrl, gzPath, navPath, progress, ct);
			if (hourly == FetchOutcome.Cancelled)
				return new DownloadResult(false, null, "Download cancelled.");
			if (hourly == FetchOutcome.Success)
				return new DownloadResult(true, navPath,
					$"Downloaded hourly {fileName} ({stamp:yyyy-MM-dd HH}:00Z)");
		}

		progress.Report("No hourly file available - falling back to daily files.");

		// ── 2) Daily files: today's may lag several hours, but better than nothing ──
		for (int daysBack = 0; daysBack <= 3; daysBack++)
		{
			if (ct.IsCancellationRequested)
				return new DownloadResult(false, null, "Download cancelled.");

			var date = DateTime.UtcNow.Date.AddDays(-daysBack);
			int doy = date.DayOfYear;
			string yy = date.ToString("yy");
			string doyStr = doy.ToString("D3");
			string fileName = $"brdc{doyStr}0.{yy}n";
			string gzPath = Path.Combine(DownloadFolder, fileName + ".gz");
			string navPath = Path.Combine(DownloadFolder, fileName);
			string ftpUrl = $"ftp://{FtpHost}" + string.Format(DailyPathFmt, date.Year, doyStr, yy);

			// Today's daily file is still being appended to on the server, so a
			// cached copy is very likely stale. Only trust the cache for days that
			// have already completed.
			if (File.Exists(navPath) && daysBack > 0)
			{
				progress.Report($"Using cached file: {navPath}");
				return new DownloadResult(true, navPath, $"Loaded from cache: {fileName}");
			}

			progress.Report(daysBack == 0
				? $"Trying today's daily file ({date:yyyy-MM-dd}, DOY {doy})..."
				: $"Not found - trying {date:yyyy-MM-dd} (DOY {doy})...");

			var daily = await TryFetchAsync(ftpUrl, gzPath, navPath, progress, ct);
			if (daily == FetchOutcome.Cancelled)
				return new DownloadResult(false, null, "Download cancelled.");
			if (daily == FetchOutcome.Success)
				return new DownloadResult(true, navPath, $"Downloaded {fileName} ({date:yyyy-MM-dd})");

			// Server copy unavailable but we have an older cached copy of today: use it.
			if (File.Exists(navPath))
			{
				progress.Report($"Using cached file: {navPath}");
				return new DownloadResult(true, navPath, $"Loaded from cache: {fileName}");
			}
		}

		return new DownloadResult(false, null,
			"Could not find a RINEX file on CDDIS FTPS (tried hourly and daily). " +
			"Check your internet connection.");
	}

	private enum FetchOutcome { Success, NotAvailable, Cancelled }

	/// <summary>
	/// Downloads a single .gz from CDDIS over explicit FTPS and decompresses it.
	/// Returns NotAvailable (rather than throwing) when the file is not published yet.
	/// </summary>
	private static async Task<FetchOutcome> TryFetchAsync(
		string ftpUrl, string gzPath, string navPath,
		IProgress<string> progress, CancellationToken ct)
	{
		TryDelete(gzPath);

		try
		{
			// Explicit FTPS: --ftp-ssl --ssl-reqd, anonymous, skip cert (-k),
			// silent (-s), fail on server error (--fail), output to file (-o)
			string args = $"--ftp-ssl --ssl-reqd -k -s --fail" +
						  $" -u \"anonymous:anonymous@\"" +
						  $" \"{ftpUrl}\"" +
						  $" -o \"{gzPath}\"";

			progress.Report($"  Connecting to {FtpHost}...");

			var (exitCode, stderr) = await RunCurlAsync(args, ct);

			if (exitCode != 0)
			{
				TryDelete(gzPath);
				string detail = string.IsNullOrWhiteSpace(stderr) ? "" : ": " + stderr.Trim();
				progress.Report($"  curl exit {exitCode}{detail} - skipping.");
				return FetchOutcome.NotAvailable;
			}

			if (!File.Exists(gzPath) || new FileInfo(gzPath).Length < 1000)
			{
				TryDelete(gzPath);
				progress.Report("  Downloaded file too small - skipping.");
				return FetchOutcome.NotAvailable;
			}

			progress.Report("  Decompressing...");

			await using (var gz = new GZipStream(File.OpenRead(gzPath), CompressionMode.Decompress))
			await using (var out_ = File.Create(navPath))
				await gz.CopyToAsync(out_, ct);

			File.Delete(gzPath);

			progress.Report($"  Saved to {navPath}");
			return FetchOutcome.Success;
		}
		catch (OperationCanceledException)
		{
			TryDelete(gzPath);
			TryDelete(navPath);
			return FetchOutcome.Cancelled;
		}
		catch (Exception ex)
		{
			TryDelete(gzPath);
			progress.Report($"  Error: {ex.Message}");
			return FetchOutcome.NotAvailable;
		}
	}

	private static Task<(int exitCode, string stderr)> RunCurlAsync(string args, CancellationToken ct)
	{
		var tcs = new TaskCompletionSource<(int, string)>(TaskCreationOptions.RunContinuationsAsynchronously);

		var psi = new ProcessStartInfo(CurlExe, args)
		{
			UseShellExecute = false,
			CreateNoWindow = true,
			RedirectStandardOutput = false,
			RedirectStandardError = true,
		};

		var proc = new Process { StartInfo = psi, EnableRaisingEvents = true };

		proc.Exited += (_, _) =>
		{
			string err = proc.StandardError.ReadToEnd();
			tcs.TrySetResult((proc.ExitCode, err));
			proc.Dispose();
		};

		ct.Register(() =>
		{
			try { if (!proc.HasExited) proc.Kill(); } catch { }
			tcs.TrySetCanceled(ct);
		});

		proc.Start();

		return tcs.Task;
	}

	private static void TryDelete(string path)
	{
		try { if (File.Exists(path)) File.Delete(path); } catch { /* ignore */ }
	}
}