namespace GPSSimulator.GpsEngine;

/// <summary>
/// Loads a route from any supported file format, choosing the parser by extension.
/// </summary>
public static class RouteLoader
{
	/// <summary>File-extension filter values for the platform file picker.</summary>
	public static readonly string[] SupportedExtensions = [".json", ".gpx"];

	/// <summary>
	/// Parse a route file. Supports trip JSON (.json) and GPX (.gpx).
	/// </summary>
	/// <exception cref="NotSupportedException">Thrown for unrecognised extensions.</exception>
	public static async Task<Trip> LoadAsync(string filePath)
	{
		string ext = Path.GetExtension(filePath).ToLowerInvariant();

		return ext switch
		{
			".gpx"  => await GpxParser.ParseFileAsync(filePath),
			".json" => await JsonTripParser.ParseFileAsync(filePath),
			_       => throw new NotSupportedException(
						   $"Unsupported route file type '{ext}'. Expected .json or .gpx.")
		};
	}
}
