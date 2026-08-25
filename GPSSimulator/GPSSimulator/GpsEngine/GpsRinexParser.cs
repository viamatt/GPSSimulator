using System.Globalization;
using static GPSSimulator.GpsEngine.GpsConstants;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// Parses RINEX 2.11 GPS navigation message files.
/// Ported from readRinexNavAll() in gpssim.c (osqzss/gps-sdr-sim).
/// </summary>
public static class GpsRinexParser
{
	/// <summary>
	/// Read all ephemeris sets from a RINEX 2.11 navigation file.
	/// Returns the number of ephemeris epochs read, or -1 on error.
	/// </summary>
	public static int ReadRinexNavAll(
		Ephemeris[][] eph,    // [EphemArraySize][MaxSat]
		IonoUtc ionoutc,
		string fname)
	{
		// Clear valid flags
		for (int i = 0; i < EphemArraySize; i++)
			for (int sv = 0; sv < MaxSat; sv++)
				eph[i][sv].Valid = false;

		string[] lines;
		try { lines = File.ReadAllLines(fname); }
		catch { return -1; }

		int lineIdx = 0;
		int ieph = 0;
		GpsTime g0 = GpsTime.Zero;
		bool headerDone = false;

		// ── Parse header ───────────────────────────────────────────────
		while (lineIdx < lines.Length)
		{
			string line = lines[lineIdx++];
			if (line.Length < 60) { if (line.Contains("END OF HEADER")) { headerDone = true; break; } continue; }

			string label = line.Length >= 73 ? line.Substring(60, Math.Min(13, line.Length - 60)).TrimEnd() : "";

			if (label == "END OF HEADER") { headerDone = true; break; }
			if (label.StartsWith("ION ALPHA"))
			{
				ionoutc.Alpha0 = ParseRinexD(line,  2, 12);
				ionoutc.Alpha1 = ParseRinexD(line, 14, 12);
				ionoutc.Alpha2 = ParseRinexD(line, 26, 12);
				ionoutc.Alpha3 = ParseRinexD(line, 38, 12);
				ionoutc.Enable = true;
			}
			else if (label.StartsWith("ION BETA"))
			{
				ionoutc.Beta0 = ParseRinexD(line,  2, 12);
				ionoutc.Beta1 = ParseRinexD(line, 14, 12);
				ionoutc.Beta2 = ParseRinexD(line, 26, 12);
				ionoutc.Beta3 = ParseRinexD(line, 38, 12);
			}
			else if (label.StartsWith("DELTA-UTC"))
			{
				ionoutc.A0  = ParseRinexD(line,  3, 19);
				ionoutc.A1  = ParseRinexD(line, 22, 19);
				ionoutc.Tot = (int)ParseRinexD(line, 41, 9);
				ionoutc.Wnt = (int)ParseRinexD(line, 50, 9);
				ionoutc.Vflg = true;
			}
			else if (label.StartsWith("LEAP SECONDS"))
			{
				ionoutc.Dtls = (int)ParseRinexD(line, 0, 6);
			}
		}

		if (!headerDone)
		{
			// Try to continue anyway - some files have no "END OF HEADER"
			lineIdx = 0;
		}

		// ── Parse navigation records ────────────────────────────────────
		while (lineIdx < lines.Length)
		{
			string line = lines[lineIdx];
			if (string.IsNullOrWhiteSpace(line)) { lineIdx++; continue; }

			// PRN and epoch line
			if (line.Length < 22) { lineIdx++; continue; }

			int sv;
			if (!int.TryParse(line.Substring(0, 2).Trim(), out sv) || sv < 1 || sv > MaxSat)
			{ lineIdx++; continue; }
			sv--; // 0-based

			if (ieph >= EphemArraySize) break;

			// If this SV is already present in the current ephemeris slot,
			// start a new slot before writing the new record (gpssim-style behavior).
			if (eph[ieph][sv].Valid && ieph + 1 < EphemArraySize)
			{
				ieph++;
				for (int s2 = 0; s2 < MaxSat; s2++)
					eph[ieph][s2] = CloneEphemeris(eph[ieph - 1][s2]);
			}

			var e = eph[ieph][sv];
			e.Valid = false;

			// Parse epoch: YY MM DD HH MM SS.S
			try
			{
				int yr  = int.Parse(line.Substring(3,  2).Trim());
				int mo  = int.Parse(line.Substring(6,  2).Trim());
				int day = int.Parse(line.Substring(9,  2).Trim());
				int hh  = int.Parse(line.Substring(12, 2).Trim());
				int mm  = int.Parse(line.Substring(15, 2).Trim());
				double sec = double.Parse(line.Substring(17, 5).Trim(), CultureInfo.InvariantCulture);
				if (yr < 80) yr += 2000; else yr += 1900;
				e.T = new GpsDateTime { Y = yr, M = mo, D = day, Hh = hh, Mm = mm, Sec = sec };
				GpsMath.Date2Gps(e.T, out e.Toc);

				e.Af0 = ParseRinexD(line, 22, 19);
				e.Af1 = ParseRinexD(line, 41, 19);
				e.Af2 = ParseRinexD(line, 60, 19);
			}
			catch { lineIdx++; continue; }

			lineIdx++;
			if (lineIdx >= lines.Length) break;

			// Broadcast Orbit - 1
			line = lines[lineIdx++];
			e.Iode  = (int)ParseRinexD(line,  3, 19);
			e.Crs   = ParseRinexD(line, 22, 19);
			e.Deltan= ParseRinexD(line, 41, 19);
			e.M0    = ParseRinexD(line, 60, 19);
			if (lineIdx >= lines.Length) continue;

			// Broadcast Orbit - 2
			line = lines[lineIdx++];
			e.Cuc   = ParseRinexD(line,  3, 19);
			e.Ecc   = ParseRinexD(line, 22, 19);
			e.Cus   = ParseRinexD(line, 41, 19);
			e.Sqrta = ParseRinexD(line, 60, 19);
			if (lineIdx >= lines.Length) continue;

			// Broadcast Orbit - 3
			line = lines[lineIdx++];
			e.Toe.Sec = ParseRinexD(line, 3, 19);
			e.Cic   = ParseRinexD(line, 22, 19);
			e.Omg0  = ParseRinexD(line, 41, 19);
			e.Cis   = ParseRinexD(line, 60, 19);
			if (lineIdx >= lines.Length) continue;

			// Broadcast Orbit - 4
			line = lines[lineIdx++];
			e.Inc0  = ParseRinexD(line,  3, 19);
			e.Crc   = ParseRinexD(line, 22, 19);
			e.Aop   = ParseRinexD(line, 41, 19);
			e.Omgdot= ParseRinexD(line, 60, 19);
			if (lineIdx >= lines.Length) continue;

			// Broadcast Orbit - 5
			line = lines[lineIdx++];
			e.Idot   = ParseRinexD(line,  3, 19);
			e.CodeL2 = (int)ParseRinexD(line, 22, 19);
			e.Toe.Week = (int)ParseRinexD(line, 41, 19);
			if (lineIdx >= lines.Length) continue;

			// Broadcast Orbit - 6
			line = lines[lineIdx++];
			e.Svhlth = (int)ParseRinexD(line, 22, 19);
			e.Tgd = ParseRinexD(line, 41, 19);
			e.Iodc = (int)ParseRinexD(line, 60, 19);
			if (lineIdx >= lines.Length) continue;

			// Broadcast Orbit - 7 (skip)
			lineIdx++;

			// Mark valid and compute derived quantities
			e.Valid = true;
			e.A       = e.Sqrta * e.Sqrta;
			e.N       = Math.Sqrt(GpsConstants.GmEarth / (e.A * e.A * e.A)) + e.Deltan;
			e.Sq1e2   = Math.Sqrt(1.0 - e.Ecc * e.Ecc);
			e.Omgkdot = e.Omgdot - GpsConstants.OmegaEarth;

			// Slot advancement is handled before parsing when duplicate SV is detected.
		}

		return ieph + 1;
	}

	private static Ephemeris CloneEphemeris(Ephemeris src)
	{
		return new Ephemeris
		{
			Valid = src.Valid,
			T = src.T,
			Toc = src.Toc,
			Toe = src.Toe,
			Iodc = src.Iodc,
			Iode = src.Iode,
			Deltan = src.Deltan,
			Cuc = src.Cuc,
			Cus = src.Cus,
			Cic = src.Cic,
			Cis = src.Cis,
			Crc = src.Crc,
			Crs = src.Crs,
			Ecc = src.Ecc,
			Sqrta = src.Sqrta,
			M0 = src.M0,
			Omg0 = src.Omg0,
			Inc0 = src.Inc0,
			Aop = src.Aop,
			Omgdot = src.Omgdot,
			Idot = src.Idot,
			Af0 = src.Af0,
			Af1 = src.Af1,
			Af2 = src.Af2,
			Tgd = src.Tgd,
			Svhlth = src.Svhlth,
			CodeL2 = src.CodeL2,
			N = src.N,
			Sq1e2 = src.Sq1e2,
			A = src.A,
			Omgkdot = src.Omgkdot
		};
	}

	// ── RINEX field parser ─────────────────────────────────────────────

	/// <summary>Parse a RINEX floating-point field (handles 'D' exponent)</summary>
	private static double ParseRinexD(string line, int start, int len)
	{
		if (start >= line.Length) return 0.0;
		int end = Math.Min(start + len, line.Length);
		string s = line.Substring(start, end - start).Replace('D', 'E').Replace('d', 'e').Trim();
		if (string.IsNullOrEmpty(s)) return 0.0;
		if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out double v)) return v;
		return 0.0;
	}
}
