using System.Diagnostics;
using System.IO.Compression;

namespace GPSSimulator.Services;

/// <summary>
/// Downloads the latest IGS broadcast RINEX 2.11 GPS nav file from NASA CDDIS FTPS.
/// Uses Windows built-in curl.exe for explicit TLS FTPS (port 21, anonymous login).
/// URL: ftp://gdc.cddis.eosdis.nasa.gov/pub/gps/data/daily/{year}/brdc/brdc{doy}0.{yy}n.gz
/// Tries today → yesterday → 2 days ago → 3 days ago as automatic fallbacks.
/// </summary>
public class RinexDownloadService
{
	private const string FtpHost = "gdc.cddis.eosdis.nasa.gov";
	private const string FtpPathFmt = "/pub/gps/data/daily/{0}/brdc/brdc{1}0.{2}n.gz";

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

		for (int daysBack = 0; daysBack <= 3; daysBack++)
		{
			var date = DateTime.UtcNow.Date.AddDays(-daysBack);
			int doy = date.DayOfYear;
			int year = date.Year;
			string yy = date.ToString("yy");
			string doyStr = doy.ToString("D3");
			string fileName = $"brdc{doyStr}0.{yy}n";
			string gzPath = Path.Combine(DownloadFolder, fileName + ".gz");
			string navPath = Path.Combine(DownloadFolder, fileName);
			string ftpUrl = $"ftp://{FtpHost}" + string.Format(FtpPathFmt, year, doyStr, yy);

			if (File.Exists(navPath))
			{
				progress.Report($"Using cached file: {navPath}");
				return new DownloadResult(true, navPath, $"Loaded from cache: {fileName}");
			}

			progress.Report(daysBack == 0
				? $"Trying today's file ({date:yyyy-MM-dd}, DOY {doy})..."
				: $"Not found — trying {date:yyyy-MM-dd} (DOY {doy})...");

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
					progress.Report($"  curl exit {exitCode}{detail} — skipping.");
					continue;
				}

				if (!File.Exists(gzPath) || new FileInfo(gzPath).Length < 1000)
				{
					TryDelete(gzPath);
					progress.Report("  Downloaded file too small — skipping.");
					continue;
				}

				progress.Report("  Decompressing...");

				await using (var gz = new GZipStream(File.OpenRead(gzPath), CompressionMode.Decompress))
				await using (var out_ = File.Create(navPath))
					await gz.CopyToAsync(out_, ct);

				File.Delete(gzPath);

				progress.Report($"  Saved to {navPath}");
				return new DownloadResult(true, navPath, $"Downloaded {fileName} ({date:yyyy-MM-dd})");
			}
			catch (OperationCanceledException)
			{
				TryDelete(gzPath);
				TryDelete(navPath);
				return new DownloadResult(false, null, "Download cancelled.");
			}
			catch (Exception ex)
			{
				TryDelete(gzPath);
				progress.Report($"  Error: {ex.Message}");
			}
		}

		return new DownloadResult(false, null,
			"Could not find a RINEX file for the last 4 days on CDDIS FTPS. " +
			"Check your internet connection.");
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