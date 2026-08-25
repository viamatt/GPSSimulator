using FrozenNorth.Gpx;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// Parses a GPX 1.0/1.1 file into the same <see cref="Trip"/> model used for
/// JSON trips, so the rest of the pipeline is format-agnostic.
///
/// Parsing is delegated to the FrozenNorth.Gpx library, which understands the
/// full GPX document structure (tracks -> segments -> points, routes, and
/// waypoints) rather than treating the file as a flat list of coordinates.
///
/// GPX differs from the trip JSON format in three important ways:
///
///  1. Timestamps are OPTIONAL. Tracks exported from route planners usually have
///     none at all. When they are missing, timings are synthesised by walking the
///     track and assuming a constant ground speed (see AssumedSpeedKph).
///  2. Speed and heading are NOT stored. Both are derived from consecutive
///     points: speed from distance/time, heading as the initial great-circle
///     bearing towards the next point.
///  3. Point density is typically far lower (a handful of points per kilometre,
///     versus 1 Hz logging). The replay path interpolates linearly between
///     points, so sparse tracks still produce smooth motion, but corners will be
///     cut. Densify() inserts intermediate points to bound that error.
/// </summary>
public static class GpxParser
{
	/// <summary>Ground speed assumed when the GPX file has no usable timestamps.</summary>
	public const double AssumedSpeedKph = 50.0;

	/// <summary>
	/// Maximum spacing (metres) between successive points after densification.
	/// Sparse tracks are subdivided so that interpolated motion stays smooth and
	/// the simulated position never makes a large instantaneous jump.
	/// </summary>
	private const double MaxPointSpacingMetres = 25.0;

	/// <summary>
	/// Gap (metres) between the end of one segment and the start of the next that
	/// is treated as a genuine break in the recording (a pause, or a stop with the
	/// receiver switched off) rather than continuous travel.
	/// </summary>
	private const double SegmentJoinThresholdMetres = 250.0;

	/// <summary>
	/// Time allowed to traverse a segment join, so the simulated vehicle neither
	/// teleports across the gap nor crawls across it for the length of the pause.
	/// </summary>
	private const double SegmentJoinSeconds = 5.0;

	private const double EarthRadiusMetres = 6_371_000.0;

	/// <summary>A single source coordinate before timing is resolved.</summary>
	private readonly record struct RawPoint(double Lat, double Lon, double Ele, DateTime? Time);

	/// <summary>
	/// Parse a GPX file and return a normalised <see cref="Trip"/>.
	/// </summary>
	/// <exception cref="InvalidDataException">Thrown when the file has no usable track points.</exception>
	public static async Task<Trip> ParseFileAsync(string filePath)
	{
		if (!File.Exists(filePath))
			throw new FileNotFoundException($"GPX file not found: {filePath}");

		// GpxReader.Load is synchronous; keep the caller's async contract and the
		// UI responsive by running it off the calling thread.
		var gpx = await Task.Run(() => GpxReader.Load(filePath))
				  ?? throw new InvalidDataException("GPX file is empty, malformed, or could not be read.");

		var segments = ExtractSegments(gpx);

		if (segments.Count == 0)
			throw new InvalidDataException(
				"GPX file contains no usable track, route or waypoint data.");

		int totalPoints = segments.Sum(s => s.Count);
		if (totalPoints < 2)
			throw new InvalidDataException(
				"GPX file contains only one point; at least two are needed to build a route.");

		// A file is only treated as timed if EVERY point carries a timestamp and
		// the sequence actually advances. Partially-timed files are unreliable, so
		// they fall back to synthesised timing.
		bool allTimed = segments.All(s => s.All(p => p.Time.HasValue));

		if (allTimed)
		{
			// Order points within each segment, then order the segments themselves
			// by their first timestamp, so out-of-order tracks still replay in the
			// correct sequence. Sorting is deliberately NOT done across the whole
			// file at once, which would interleave separate segments.
			foreach (var seg in segments)
				seg.Sort((a, b) => a.Time!.Value.CompareTo(b.Time!.Value));

			segments.Sort((a, b) => a[0].Time!.Value.CompareTo(b[0].Time!.Value));
		}

		var flat    = new List<RawPoint>(totalPoints);
		var offsets = new List<double>(totalPoints);

		if (allTimed)
		{
			var origin = segments[0][0].Time!.Value;
			foreach (var seg in segments)
				foreach (var p in seg)
				{
					flat.Add(p);
					offsets.Add((p.Time!.Value - origin).TotalSeconds);
				}

			// Guard against duplicate/backwards timestamps, which would create a
			// zero-length segment and hence an infinite computed speed.
			for (int i = 1; i < offsets.Count; i++)
				if (offsets[i] <= offsets[i - 1])
					offsets[i] = offsets[i - 1] + 0.1;
		}
		else
		{
			// No usable timestamps: walk the track at a constant assumed speed.
			double mps = AssumedSpeedKph / 3.6;

			foreach (var seg in segments)
			{
				foreach (var p in seg)
				{
					if (flat.Count == 0)
					{
						flat.Add(p);
						offsets.Add(0.0);
						continue;
					}

					var prev = flat[^1];
					double d = HaversineMetres(prev.Lat, prev.Lon, p.Lat, p.Lon);
					offsets.Add(offsets[^1] + Math.Max(d / mps, 0.1));
					flat.Add(p);
				}
			}
		}

		ApplySegmentJoinTiming(segments, flat, offsets);

		bool timedUsable = allTimed && offsets[^1] > 0.5;
		var  startUtc    = timedUsable ? flat[0].Time!.Value : DateTime.UtcNow;

		// Build points, deriving speed and heading from the geometry.
		var points = new List<TripPoint>(flat.Count);
		for (int i = 0; i < flat.Count; i++)
		{
			// Speed over the segment ENDING at this point (first point borrows the
			// second segment's speed so the track doesn't start at 0 km/h).
			int a = i == 0 ? 0 : i - 1;
			int b = i == 0 ? 1 : i;
			double segMetres = HaversineMetres(flat[a].Lat, flat[a].Lon, flat[b].Lat, flat[b].Lon);
			double segSecs   = Math.Max(offsets[b] - offsets[a], 1e-6);
			double speedKph  = segMetres / segSecs * 3.6;

			// Heading towards the NEXT point (last point keeps the previous heading).
			int h0 = i < flat.Count - 1 ? i : i - 1;
			int h1 = i < flat.Count - 1 ? i + 1 : i;
			double heading = InitialBearingDeg(flat[h0].Lat, flat[h0].Lon, flat[h1].Lat, flat[h1].Lon);

			points.Add(new TripPoint(
				OccurredAtUtc : startUtc.AddSeconds(offsets[i]),
				Latitude      : flat[i].Lat,
				Longitude     : flat[i].Lon,
				AltitudeMeters: flat[i].Ele,
				SpeedKph      : Math.Round(speedKph, 2),
				HeadingDeg    : (int)Math.Round(heading)
			) { OffsetSeconds = offsets[i] });
		}

		points = Densify(points);

		string detail = segments.Count > 1
			? $"GPX ({segments.Count} segments)"
			: "GPX";

		return new Trip
		{
			Points              = points,
			StartUtc            = points[0].OccurredAtUtc,
			TotalDuration       = TimeSpan.FromSeconds(points[^1].OffsetSeconds),
			TimingIsSynthesized = !timedUsable,
			SourceFormat        = detail,
		};
	}

	/// <summary>
	/// Pulls ordered coordinate lists out of the GPX document, preserving segment
	/// boundaries.
	///
	/// Sources are considered in priority order and are NOT merged. A file holding
	/// both a detailed track and a pair of start/finish waypoints must replay the
	/// track; mixing the two produced a straight line from start to end.
	/// </summary>
	private static List<List<RawPoint>> ExtractSegments(Gpx gpx)
	{
		var segments = new List<List<RawPoint>>();

		// 1. Tracks are the richest source: each track may hold several segments,
		//    and a file may hold several tracks. Keep every segment separate.
		if (gpx.Tracks != null)
		{
			foreach (var track in gpx.Tracks)
			{
				if (track?.Segments == null) continue;

				foreach (var seg in track.Segments)
				{
					var pts = Convert(seg?.Points);
					if (pts.Count > 0) segments.Add(pts);
				}
			}
		}

		// 2. Fall back to routes (planned rather than recorded).
		if (segments.Count == 0 && gpx.Routes != null)
		{
			foreach (var route in gpx.Routes)
			{
				var pts = Convert(route?.Points);
				if (pts.Count > 0) segments.Add(pts);
			}
		}

		// 3. Last resort: standalone waypoints, treated as one ordered list.
		if (segments.Count == 0)
		{
			var pts = Convert(gpx.Waypoints);
			if (pts.Count > 0) segments.Add(pts);
		}

		return segments;
	}

	/// <summary>
	/// Converts library points to <see cref="RawPoint"/>, dropping any with
	/// out-of-range coordinates.
	/// </summary>
	private static List<RawPoint> Convert(IEnumerable<GpxPoint>? source)
	{
		var result = new List<RawPoint>();
		if (source == null) return result;

		foreach (var p in source)
		{
			if (p == null) continue;
			if (double.IsNaN(p.Latitude) || double.IsNaN(p.Longitude)) continue;
			if (p.Latitude < -90 || p.Latitude > 90) continue;
			if (p.Longitude < -180 || p.Longitude > 180) continue;

			DateTime? time = p.Time.HasValue ? p.Time.Value.ToUniversalTime() : null;

			result.Add(new RawPoint(p.Latitude, p.Longitude, p.Elevation ?? 0.0, time));
		}

		return result;
	}

	/// <summary>
	/// Where consecutive segments are separated by a large distance, allots the
	/// crossing a fixed short time. Without this, a recording paused for an hour
	/// and resumed a mile away would either jump instantly (synthesised timing) or
	/// crawl across the gap for the whole hour (recorded timing), and the densifier
	/// would fill the join with points strung along a straight line.
	/// </summary>
	private static void ApplySegmentJoinTiming(
		List<List<RawPoint>> segments, List<RawPoint> flat, List<double> offsets)
	{
		if (segments.Count < 2) return;

		// Index of the first point of each segment after the first.
		var joinIndices = new List<int>();
		int index = 0;
		for (int s = 0; s < segments.Count - 1; s++)
		{
			index += segments[s].Count;
			joinIndices.Add(index);
		}

		foreach (int j in joinIndices)
		{
			if (j <= 0 || j >= flat.Count) continue;

			double gap = HaversineMetres(
				flat[j - 1].Lat, flat[j - 1].Lon, flat[j].Lat, flat[j].Lon);

			if (gap < SegmentJoinThresholdMetres) continue;

			double current = offsets[j] - offsets[j - 1];
			double shift   = SegmentJoinSeconds - current;
			if (Math.Abs(shift) < 1e-6) continue;

			// Re-time the join and slide the remainder of the trip to match.
			for (int i = j; i < offsets.Count; i++)
				offsets[i] += shift;
		}
	}

	/// <summary>
	/// Inserts intermediate points so no two consecutive points are further apart
	/// than <see cref="MaxPointSpacingMetres"/>. GPX tracks are often very sparse;
	/// without this a single long leg becomes one straight interpolated line, and
	/// the UI progress/telemetry updates only once for the whole leg.
	/// </summary>
	private static List<TripPoint> Densify(List<TripPoint> src)
	{
		if (src.Count < 2) return src;

		var result = new List<TripPoint>(src.Count);

		for (int i = 0; i < src.Count - 1; i++)
		{
			var a = src[i];
			var b = src[i + 1];
			result.Add(a);

			double metres = HaversineMetres(a.Latitude, a.Longitude, b.Latitude, b.Longitude);
			int subdivisions = (int)(metres / MaxPointSpacingMetres);
			if (subdivisions <= 1) continue;

			// Cap the work done for pathological files (e.g. two points 500 km apart).
			subdivisions = Math.Min(subdivisions, 2000);

			for (int s = 1; s < subdivisions; s++)
			{
				double f = (double)s / subdivisions;
				double offset = a.OffsetSeconds + (b.OffsetSeconds - a.OffsetSeconds) * f;

				result.Add(new TripPoint(
					OccurredAtUtc : a.OccurredAtUtc.AddSeconds(offset - a.OffsetSeconds),
					Latitude      : a.Latitude       + (b.Latitude       - a.Latitude)       * f,
					Longitude     : a.Longitude      + (b.Longitude      - a.Longitude)      * f,
					AltitudeMeters: a.AltitudeMeters + (b.AltitudeMeters - a.AltitudeMeters) * f,
					SpeedKph      : b.SpeedKph,
					HeadingDeg    : b.HeadingDeg
				) { OffsetSeconds = offset });
			}
		}

		result.Add(src[^1]);
		return result;
	}

	// ── Geodesy helpers ──────────────────────────────────────────────────────

	/// <summary>Great-circle distance in metres between two WGS-84 points.</summary>
	private static double HaversineMetres(double lat1, double lon1, double lat2, double lon2)
	{
		double p1 = lat1 * Math.PI / 180.0;
		double p2 = lat2 * Math.PI / 180.0;
		double dp = (lat2 - lat1) * Math.PI / 180.0;
		double dl = (lon2 - lon1) * Math.PI / 180.0;

		double a = Math.Sin(dp / 2) * Math.Sin(dp / 2) +
				   Math.Cos(p1) * Math.Cos(p2) * Math.Sin(dl / 2) * Math.Sin(dl / 2);

		return 2.0 * EarthRadiusMetres * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1.0 - a));
	}

	/// <summary>Initial great-circle bearing in degrees (0-359, 0 = north).</summary>
	private static double InitialBearingDeg(double lat1, double lon1, double lat2, double lon2)
	{
		double p1 = lat1 * Math.PI / 180.0;
		double p2 = lat2 * Math.PI / 180.0;
		double dl = (lon2 - lon1) * Math.PI / 180.0;

		double y = Math.Sin(dl) * Math.Cos(p2);
		double x = Math.Cos(p1) * Math.Sin(p2) - Math.Sin(p1) * Math.Cos(p2) * Math.Cos(dl);

		double deg = Math.Atan2(y, x) * 180.0 / Math.PI;
		return (deg + 360.0) % 360.0;
	}
}
