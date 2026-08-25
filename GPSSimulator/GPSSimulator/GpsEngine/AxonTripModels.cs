using System.Text.Json.Serialization;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// A single position point extracted from an AxonTrip JSON file.
/// All fields map directly to the JSON Positions array entries.
/// </summary>
public record TripPoint(
	DateTime OccurredAtUtc,
	double   Latitude,
	double   Longitude,
	double   AltitudeMeters,
	double   SpeedKph,
	int      HeadingDeg
)
{
	/// <summary>Offset from the first trip point (seconds).</summary>
	public double OffsetSeconds { get; init; }
}

/// <summary>
/// Parsed AxonTrip result: ordered trip points with metadata.
/// </summary>
public class AxonTrip
{
	public List<TripPoint> Points       { get; init; } = [];
	public DateTime        StartUtc     { get; init; }
	public TimeSpan        TotalDuration { get; init; }
	public int             PointCount   => Points.Count;

	/// <summary>Human-readable summary for the log.</summary>
	public string Summary =>
		$"{PointCount} points over {TotalDuration:hh\\:mm\\:ss}, " +
		$"start {StartUtc:yyyy-MM-dd HH:mm:ss}Z";
}

// ── Internal JSON deserialization types ──────────────────────────────────────

internal sealed class AxonTripRoot
{
	[JsonPropertyName("Positions")]
	public List<AxonPositionRaw> Positions { get; set; } = [];
}

internal sealed class AxonPositionRaw
{
	[JsonPropertyName("OccurredAt")]
	public string OccurredAt { get; set; } = string.Empty;

	[JsonPropertyName("Latitude")]
	public double Latitude { get; set; }

	[JsonPropertyName("Longitude")]
	public double Longitude { get; set; }

	[JsonPropertyName("Altitude")]
	public double Altitude { get; set; }

	[JsonPropertyName("SpeedKph")]
	public double SpeedKph { get; set; }

	[JsonPropertyName("Heading")]
	public int Heading { get; set; }

	[JsonPropertyName("AccuracyMeters")]
	public double AccuracyMeters { get; set; }
}
