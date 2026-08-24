using static GPSSimulator.GpsEngine.GpsConstants;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// GPS math utilities: coordinate transforms, satellite position, ionospheric delay.
/// Ported directly from gpssim.c (osqzss/gps-sdr-sim).
/// </summary>
public static class GpsMath
{
	// ── Vector helpers ──────────────────────────────────────────────────────

	public static double NormVect(double[] x) =>
		Math.Sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);

	public static double DotProd(double[] x1, double[] x2) =>
		x1[0] * x2[0] + x1[1] * x2[1] + x1[2] * x2[2];

	public static void SubVect(double[] y, double[] x1, double[] x2)
	{
		y[0] = x1[0] - x2[0];
		y[1] = x1[1] - x2[1];
		y[2] = x1[2] - x2[2];
	}

	// ── Coordinate transforms ────────────────────────────────────────────

	/// <summary>Convert ECEF XYZ to geodetic lat/lon/height (radians, metres)</summary>
	public static void Xyz2Llh(double[] xyz, double[] llh)
	{
		const double a = Wgs84Radius;
		const double e = Wgs84Eccentricity;
		const double eps = 1.0e-3;
		const double e2 = e * e;

		if (NormVect(xyz) < eps)
		{
			llh[0] = 0; llh[1] = 0; llh[2] = -a;
			return;
		}

		double x = xyz[0], y = xyz[1], z = xyz[2];
		double rho2 = x * x + y * y;
		double dz = e2 * z;

		while (true)
		{
			double zdz = z + dz;
			double nh = Math.Sqrt(rho2 + zdz * zdz);
			double slat = zdz / nh;
			double n = a / Math.Sqrt(1.0 - e2 * slat * slat);
			double dzNew = n * e2 * slat;
			if (Math.Abs(dz - dzNew) < eps) break;
			dz = dzNew;
		}

		double zdz2 = z + dz;
		double nh2 = Math.Sqrt(rho2 + zdz2 * zdz2);
		double slat2 = zdz2 / nh2;
		double n2 = a / Math.Sqrt(1.0 - e2 * slat2 * slat2);

		llh[0] = Math.Atan2(zdz2, Math.Sqrt(rho2));
		llh[1] = Math.Atan2(y, x);
		llh[2] = nh2 - n2;
	}

	/// <summary>Convert geodetic lat/lon/height (radians, metres) to ECEF XYZ</summary>
	public static void Llh2Xyz(double[] llh, double[] xyz)
	{
		const double a = Wgs84Radius;
		const double e = Wgs84Eccentricity;
		const double e2 = e * e;

		double clat = Math.Cos(llh[0]);
		double slat = Math.Sin(llh[0]);
		double clon = Math.Cos(llh[1]);
		double slon = Math.Sin(llh[1]);
		double d = e * slat;
		double n = a / Math.Sqrt(1.0 - d * d);
		double nph = n + llh[2];

		xyz[0] = nph * clat * clon;
		xyz[1] = nph * clat * slon;
		xyz[2] = (n * (1.0 - e2) + llh[2]) * slat;
	}

	/// <summary>Local tangent coordinate matrix at a geodetic point</summary>
	public static void Ltcmat(double[] llh, double[][] tmat)
	{
		double slat = Math.Sin(llh[0]);
		double clat = Math.Cos(llh[0]);
		double slon = Math.Sin(llh[1]);
		double clon = Math.Cos(llh[1]);

		tmat[0][0] = -slat * clon; tmat[0][1] = -slat * slon; tmat[0][2] = clat;
		tmat[1][0] = -slon;        tmat[1][1] = clon;          tmat[1][2] = 0.0;
		tmat[2][0] = clat * clon;  tmat[2][1] = clat * slon;  tmat[2][2] = slat;
	}

	/// <summary>ECEF vector to local North/East/Up</summary>
	public static void Ecef2Neu(double[] xyz, double[][] tmat, double[] neu)
	{
		neu[0] = tmat[0][0] * xyz[0] + tmat[0][1] * xyz[1] + tmat[0][2] * xyz[2];
		neu[1] = tmat[1][0] * xyz[0] + tmat[1][1] * xyz[1] + tmat[1][2] * xyz[2];
		neu[2] = tmat[2][0] * xyz[0] + tmat[2][1] * xyz[1] + tmat[2][2] * xyz[2];
	}

	/// <summary>NEU vector to azimuth/elevation (radians)</summary>
	public static void Neu2Azel(double[] azel, double[] neu)
	{
		double ne = Math.Sqrt(neu[0] * neu[0] + neu[1] * neu[1]);
		azel[0] = Math.Atan2(neu[1], neu[0]);
		if (azel[0] < 0.0) azel[0] += 2.0 * Pi;
		azel[1] = Math.Atan2(neu[2], ne);
	}

	// ── GPS time helpers ─────────────────────────────────────────────────

	public static double SubGpsTime(GpsTime g1, GpsTime g0)
	{
		double dt = g1.Sec - g0.Sec;
		dt += (g1.Week - g0.Week) * SecondsInWeek;
		return dt;
	}

	public static GpsTime IncGpsTime(GpsTime g0, double dt)
	{
		var g1 = new GpsTime { Week = g0.Week, Sec = g0.Sec + dt };
		g1.Sec = Math.Round(g1.Sec * 1000.0) / 1000.0;
		while (g1.Sec >= SecondsInWeek) { g1.Sec -= SecondsInWeek; g1.Week++; }
		while (g1.Sec < 0.0)           { g1.Sec += SecondsInWeek; g1.Week--; }
		return g1;
	}

	public static void Date2Gps(GpsDateTime t, out GpsTime g)
	{
		int[] doy = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
		int ye = t.Y - 1980;
		int lpdays = ye / 4 + 1;
		if ((ye % 4) == 0 && t.M <= 2) lpdays--;
		int de = ye * 365 + doy[t.M - 1] + t.D + lpdays - 6;
		g = new GpsTime
		{
			Week = de / 7,
			Sec = (de % 7) * SecondsInDay + t.Hh * SecondsInHour + t.Mm * SecondsInMinute + t.Sec
		};
	}

	// ── Satellite position ───────────────────────────────────────────────

	/// <summary>
	/// Compute satellite ECEF position, velocity, and clock bias from ephemeris.
	/// Ported from satpos() in gpssim.c.
	/// </summary>
	public static void SatPos(Ephemeris eph, GpsTime g,
							   double[] pos, double[] vel, double[] clk)
	{
		double tk = g.Sec - eph.Toe.Sec;
		if (tk > SecondsInHalfWeek)  tk -= SecondsInWeek;
		else if (tk < -SecondsInHalfWeek) tk += SecondsInWeek;

		double mk = eph.M0 + eph.N * tk;
		double ek = mk, ekOld = ek + 1.0, oneMinusCosE = 0;

		while (Math.Abs(ek - ekOld) > 1.0e-14)
		{
			ekOld = ek;
			oneMinusCosE = 1.0 - eph.Ecc * Math.Cos(ekOld);
			ek = ek + (mk - ekOld + eph.Ecc * Math.Sin(ekOld)) / oneMinusCosE;
		}

		double sek = Math.Sin(ek), cek = Math.Cos(ek);
		double ekdot = eph.N / oneMinusCosE;
		double relativistic = -4.442807633e-10 * eph.Ecc * eph.Sqrta * sek;

		double pk = Math.Atan2(eph.Sq1e2 * sek, cek - eph.Ecc) + eph.Aop;
		double pkdot = eph.Sq1e2 * ekdot / oneMinusCosE;

		double s2pk = Math.Sin(2.0 * pk), c2pk = Math.Cos(2.0 * pk);
		double uk = pk + eph.Cus * s2pk + eph.Cuc * c2pk;
		double suk = Math.Sin(uk), cuk = Math.Cos(uk);
		double ukdot = pkdot * (1.0 + 2.0 * (eph.Cus * c2pk - eph.Cuc * s2pk));

		double rk = eph.A * oneMinusCosE + eph.Crc * c2pk + eph.Crs * s2pk;
		double rkdot = eph.A * eph.Ecc * sek * ekdot + 2.0 * pkdot * (eph.Crs * c2pk - eph.Crc * s2pk);

		double ik = eph.Inc0 + eph.Idot * tk + eph.Cic * c2pk + eph.Cis * s2pk;
		double sik = Math.Sin(ik), cik = Math.Cos(ik);
		double ikdot = eph.Idot + 2.0 * pkdot * (eph.Cis * c2pk - eph.Cic * s2pk);

		double xpk = rk * cuk, ypk = rk * suk;
		double xpkdot = rkdot * cuk - ypk * ukdot;
		double ypkdot = rkdot * suk + xpk * ukdot;

		double ok = eph.Omg0 + tk * eph.Omgkdot - OmegaEarth * eph.Toe.Sec;
		double sok = Math.Sin(ok), cok = Math.Cos(ok);

		pos[0] = xpk * cok - ypk * cik * sok;
		pos[1] = xpk * sok + ypk * cik * cok;
		pos[2] = ypk * sik;

		double tmp = ypkdot * cik - ypk * sik * ikdot;
		vel[0] = -eph.Omgkdot * pos[1] + xpkdot * cok - tmp * sok;
		vel[1] =  eph.Omgkdot * pos[0] + xpkdot * sok + tmp * cok;
		vel[2] = ypk * cik * ikdot + ypkdot * sik;

		// Clock correction
		tk = g.Sec - eph.Toc.Sec;
		if (tk > SecondsInHalfWeek)  tk -= SecondsInWeek;
		else if (tk < -SecondsInHalfWeek) tk += SecondsInWeek;

		clk[0] = eph.Af0 + tk * (eph.Af1 + tk * eph.Af2) + relativistic - eph.Tgd;
		clk[1] = eph.Af1 + 2.0 * tk * eph.Af2;
	}

	// ── Ionospheric delay ────────────────────────────────────────────────

	/// <summary>Klobuchar ionospheric delay model. Returns delay in metres.</summary>
	public static double IonoDelay(IonoUtc ionoutc, GpsTime g, double[] llh, double[] azel)
	{
		if (!ionoutc.Enable) return 0.0;

		double E = azel[1] / Pi;
		double phi_u = llh[0] / Pi;
		double lam_u = llh[1] / Pi;
		double F = 1.0 + 16.0 * Math.Pow(0.53 - E, 3.0);

		if (!ionoutc.Vflg)
			return F * 5.0e-9 * SpeedOfLight;

		double psi = 0.0137 / (E + 0.11) - 0.022;
		double phi_i = phi_u + psi * Math.Cos(azel[0]);
		phi_i = Math.Clamp(phi_i, -0.416, 0.416);
		double lam_i = lam_u + psi * Math.Sin(azel[0]) / Math.Cos(phi_i * Pi);
		double phi_m = phi_i + 0.064 * Math.Cos((lam_i - 1.617) * Pi);

		double phi_m2 = phi_m * phi_m;
		double phi_m3 = phi_m2 * phi_m;

		double t = 4.32e4 * lam_i + g.Sec;
		t = t % SecondsInDay;
		if (t < 0) t += SecondsInDay;

		double amp = ionoutc.Alpha0
				   + ionoutc.Alpha1 * phi_m
				   + ionoutc.Alpha2 * phi_m2
				   + ionoutc.Alpha3 * phi_m3;
		if (amp < 0) amp = 0;

		double per = ionoutc.Beta0
				   + ionoutc.Beta1 * phi_m
				   + ionoutc.Beta2 * phi_m2
				   + ionoutc.Beta3 * phi_m3;
		if (per < 72000) per = 72000;

		double x = 2.0 * Pi * (t - 50400.0) / per;

		double iono_delay;
		if (Math.Abs(x) >= 1.57)
			iono_delay = F * 5.0e-9 * SpeedOfLight;
		else
		{
			double x2 = x * x;
			iono_delay = F * (5.0e-9 + amp * (1.0 - x2 / 2.0 + x2 * x2 / 24.0)) * SpeedOfLight;
		}

		return iono_delay;
	}

	// ── Tmat helpers ─────────────────────────────────────────────────────

	public static double[][] MakeTmat() =>
		new double[][] { new double[3], new double[3], new double[3] };
}
