using System.Text.Json.Serialization;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// A single position point on a route.
/// For JSON routes, all fields map directly to the Positions array entries.
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
/// Parsed route: ordered trip points with metadata.
/// </summary>
public class Trip
{
	public List<TripPoint> Points       { get; init; } = [];
	public DateTime        StartUtc     { get; init; }
	public TimeSpan        TotalDuration { get; init; }
	public int             PointCount   => Points.Count;

	/// <summary>
	/// True when the source file carried no usable timestamps and the timings were
	/// synthesised from an assumed constant speed. Common for GPX routes exported
	/// from planning tools. Surfaced in the UI so the user knows the replay
	/// duration is an estimate rather than a recorded fact.
	/// </summary>
	public bool TimingIsSynthesized { get; init; }

	/// <summary>Source format, for display purposes.</summary>
	public string SourceFormat { get; init; } = "JSON";

	/// <summary>Human-readable summary for the log.</summary>
	public string Summary =>
		$"{SourceFormat}: {PointCount} points over {TotalDuration:hh\\:mm\\:ss}, " +
		$"start {StartUtc:yyyy-MM-dd HH:mm:ss}Z" +
		(TimingIsSynthesized ? " (timings estimated - no timestamps in file)" : "");

	/// <summary>
	/// Linearly interpolate the trip position at <paramref name="seconds"/> past
	/// the start of the trip. Clamps to the first/last point outside the range.
	/// Returns the index of the trip point at or before the requested time.
	/// </summary>
	public int Interpolate(double seconds, out double lat, out double lon, out double alt)
	{
		if (Points.Count == 0)
		{
			lat = lon = alt = 0.0;
			return 0;
		}

		if (Points.Count == 1 || seconds <= Points[0].OffsetSeconds)
		{
			var only = Points[0];
			lat = only.Latitude; lon = only.Longitude; alt = only.AltitudeMeters;
			return 0;
		}

		if (seconds >= Points[^1].OffsetSeconds)
		{
			var last = Points[^1];
			lat = last.Latitude; lon = last.Longitude; alt = last.AltitudeMeters;
			return Points.Count - 1;
		}

		// Binary search for the segment bracketing the requested time.
		int lo = 0, hi = Points.Count - 1;
		while (hi - lo > 1)
		{
			int mid = (lo + hi) / 2;
			if (Points[mid].OffsetSeconds <= seconds) lo = mid;
			else hi = mid;
		}

		var a = Points[lo];
		var b = Points[hi];
		double span = b.OffsetSeconds - a.OffsetSeconds;
		double f = span > 1e-9 ? (seconds - a.OffsetSeconds) / span : 0.0;

		lat = a.Latitude       + (b.Latitude       - a.Latitude)       * f;
		lon = a.Longitude      + (b.Longitude      - a.Longitude)      * f;
		alt = a.AltitudeMeters + (b.AltitudeMeters - a.AltitudeMeters) * f;
		return lo;
	}
}

// ── Internal JSON deserialization types ──────────────────────────────────────

internal sealed class JsonTripRoot
{
	[JsonPropertyName("Positions")]
	public List<JsonPositionRaw> Positions { get; set; } = [];
}

internal sealed class JsonPositionRaw
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
