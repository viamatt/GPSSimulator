using System.Text.Json;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// Parses a JSON trip file and returns a normalised <see cref="Trip"/>.
/// </summary>
public static class JsonTripParser
{
	private static readonly JsonSerializerOptions _opts = new()
	{
		PropertyNameCaseInsensitive = true,
		AllowTrailingCommas = true,
		ReadCommentHandling = JsonCommentHandling.Skip,
	};

	/// <summary>
	/// Parse the given file and return a <see cref="Trip"/> with all
	/// positions sorted by time and <see cref="TripPoint.OffsetSeconds"/>
	/// relative to the first point.
	/// </summary>
	/// <exception cref="InvalidDataException">Thrown when the file has no valid positions.</exception>
	public static async Task<Trip> ParseFileAsync(string filePath)
	{
		if (!File.Exists(filePath))
			throw new FileNotFoundException($"Trip file not found: {filePath}");

		await using var stream = File.OpenRead(filePath);
		var root = await JsonSerializer.DeserializeAsync<JsonTripRoot>(stream, _opts)
				   ?? throw new InvalidDataException("Failed to deserialize trip JSON.");

		if (root.Positions.Count == 0)
			throw new InvalidDataException("Trip JSON contains no positions.");

		// Parse and sort by time
		var parsed = new List<(DateTime utc, JsonPositionRaw raw)>(root.Positions.Count);
		foreach (var p in root.Positions)
		{
			if (DateTime.TryParse(p.OccurredAt,
					null,
					System.Globalization.DateTimeStyles.AdjustToUniversal |
					System.Globalization.DateTimeStyles.AssumeUniversal,
					out var utc))
			{
				parsed.Add((utc, p));
			}
		}

		if (parsed.Count == 0)
			throw new InvalidDataException("Trip JSON: no positions with parseable timestamps.");

		parsed.Sort((a, b) => a.utc.CompareTo(b.utc));

		var origin    = parsed[0].utc;
		var lastTime  = parsed[^1].utc;
		var duration  = lastTime - origin;

		var points = parsed.Select(entry => new TripPoint(
			OccurredAtUtc  : entry.utc,
			Latitude       : entry.raw.Latitude,
			Longitude      : entry.raw.Longitude,
			AltitudeMeters : entry.raw.Altitude,
			SpeedKph       : entry.raw.SpeedKph,
			HeadingDeg     : entry.raw.Heading
		) { OffsetSeconds = (entry.utc - origin).TotalSeconds }).ToList();

		return new Trip
		{
			Points        = points,
			StartUtc      = origin,
			TotalDuration = duration,
			SourceFormat  = "JSON",
		};
	}
}
