using static GPSSimulator.GpsEngine.GpsConstants;

namespace GPSSimulator.GpsEngine;

/// <summary>
/// GPS navigation message generation: C/A code, subframe encoding, checksum.
/// Ported directly from gpssim.c (osqzss/gps-sdr-sim).
/// </summary>
public static class GpsNavMessage
{
	private static readonly int[] Delay = {
		  5,   6,   7,   8,  17,  18, 139, 140, 141, 251,
		252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
		861, 862
	};

	/// <summary>Generate C/A code sequence for PRN 1-32</summary>
	public static void Codegen(int[] ca, int prn)
	{
		if (prn < 1 || prn > 32) return;

		int[] g1 = new int[CaSeqLen], g2 = new int[CaSeqLen];
		int[] r1 = new int[10], r2 = new int[10];
		for (int i = 0; i < 10; i++) r1[i] = r2[i] = -1;

		for (int i = 0; i < CaSeqLen; i++)
		{
			g1[i] = r1[9];
			g2[i] = r2[9];
			int c1 = r1[2] * r1[9];
			int c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];
			for (int j = 9; j > 0; j--) { r1[j] = r1[j - 1]; r2[j] = r2[j - 1]; }
			r1[0] = c1;
			r2[0] = c2;
		}

		int delay = Delay[prn - 1];
		for (int i = 0, j = CaSeqLen - delay; i < CaSeqLen; i++, j++)
			ca[i] = (1 - g1[i] * g2[j % CaSeqLen]) / 2;
	}

	// ── Subframe encoding ────────────────────────────────────────────────

	/// <summary>Encode ephemeris into 5 GPS subframes (10 words each)</summary>
	public static void Eph2Sbf(Ephemeris eph, IonoUtc ionoutc, ulong[][] sbf)
	{
		ulong wn       = 0UL;
		ulong toe      = (ulong)(eph.Toe.Sec / 16.0);
		ulong toc      = (ulong)(eph.Toc.Sec / 16.0);
		ulong iode     = (ulong)eph.Iode;
		ulong iodc     = (ulong)eph.Iodc;
		long  deltan   = (long)(eph.Deltan / Pow2M43 / Pi);
		long  cuc      = (long)(eph.Cuc / Pow2M29);
		long  cus      = (long)(eph.Cus / Pow2M29);
		long  cic      = (long)(eph.Cic / Pow2M29);
		long  cis      = (long)(eph.Cis / Pow2M29);
		long  crc      = (long)(eph.Crc / Pow2M5);
		long  crs      = (long)(eph.Crs / Pow2M5);
		ulong ecc      = (ulong)(eph.Ecc / Pow2M33);
		ulong sqrta    = (ulong)(eph.Sqrta / Pow2M19);
		long  m0       = (long)(eph.M0 / Pow2M31 / Pi);
		long  omg0     = (long)(eph.Omg0 / Pow2M31 / Pi);
		long  inc0     = (long)(eph.Inc0 / Pow2M31 / Pi);
		long  aop      = (long)(eph.Aop / Pow2M31 / Pi);
		long  omgdot   = (long)(eph.Omgdot / Pow2M43 / Pi);
		long  idot     = (long)(eph.Idot / Pow2M43 / Pi);
		long  af0      = (long)(eph.Af0 / Pow2M31);
		long  af1      = (long)(eph.Af1 / Pow2M43);
		long  af2      = (long)(eph.Af2 / Pow2M55);
		long  tgd      = (long)(eph.Tgd / Pow2M31);
		ulong svhlth   = (ulong)eph.Svhlth;
		ulong codeL2   = (ulong)eph.CodeL2;
		ulong wna      = (ulong)(eph.Toe.Week % 256);
		ulong toa      = (ulong)(eph.Toe.Sec / 4096.0);
		ulong ura      = 0UL, dataId = 1UL;
		ulong sbf4p25  = 63UL, sbf5p25 = 51UL, sbf4p18 = 56UL;

		long  alpha0 = 0, alpha1 = 0, alpha2 = 0, alpha3 = 0;
		long  beta0  = 0, beta1  = 0, beta2  = 0, beta3  = 0;
		long  A0 = 0, A1 = 0, dtls = 0;
		ulong tot = 0, wnt = 0, wnlsf = 0, dtlsf = 0, dn = 0;

		if (ionoutc.Vflg)
		{
			alpha0 = (long)(ionoutc.Alpha0 / Pow2M30);
			alpha1 = (long)(ionoutc.Alpha1 / Pow2M27);
			alpha2 = (long)(ionoutc.Alpha2 / Pow2M24);
			alpha3 = (long)(ionoutc.Alpha3 / Pow2M24);
			beta0  = (long)(ionoutc.Beta0  / 2048.0);
			beta1  = (long)(ionoutc.Beta1  / 16384.0);
			beta2  = (long)(ionoutc.Beta2  / 65536.0);
			beta3  = (long)(ionoutc.Beta3  / 65536.0);
			A1     = (long)(ionoutc.A1 / Pow2M50);
			A0     = (long)(ionoutc.A0 / Pow2M30);
			dtls   = (long)ionoutc.Dtls;
			tot    = (ulong)ionoutc.Tot;
			wnt    = (ulong)ionoutc.Wnt;
			wnlsf  = (ulong)ionoutc.Wnlsf;
			dtlsf  = (ulong)ionoutc.Dtlsf;
			dn     = (ulong)ionoutc.Dn;
		}

		// Subframe 1
		sbf[0][0] = 0x8B0000UL << 6;
		sbf[0][1] = 0x1UL << 8;
		sbf[0][2] = ((wn & 0x3FFUL) << 20) | ((codeL2 & 0x3UL) << 18) | ((ura & 0xFUL) << 14)
				  | ((svhlth & 0x3FUL) << 8) | (((iodc >> 8) & 0x3UL) << 6);
		sbf[0][3] = 0UL; sbf[0][4] = 0UL; sbf[0][5] = 0UL;
		sbf[0][6] = ((ulong)(tgd & 0xFFL)) << 6;
		sbf[0][7] = ((iodc & 0xFFUL) << 22) | ((toc & 0xFFFFUL) << 6);
		sbf[0][8] = (((ulong)(af2 & 0xFFL)) << 22) | (((ulong)(af1 & 0xFFFFL)) << 6);
		sbf[0][9] = ((ulong)(af0 & 0x3FFFFFL)) << 8;

		// Subframe 2
		sbf[1][0] = 0x8B0000UL << 6;
		sbf[1][1] = 0x2UL << 8;
		sbf[1][2] = ((iode & 0xFFUL) << 22) | (((ulong)(crs & 0xFFFFL)) << 6);
		sbf[1][3] = (((ulong)(deltan & 0xFFFFL)) << 14) | (((ulong)((m0 >> 24) & 0xFFL)) << 6);
		sbf[1][4] = ((ulong)(m0 & 0xFFFFFFL)) << 6;
		sbf[1][5] = (((ulong)(cuc & 0xFFFFL)) << 14) | (((ulong)((ecc >> 24) & 0xFFUL)) << 6);
		sbf[1][6] = ((ecc & 0xFFFFFFUL) << 6);
		sbf[1][7] = (((ulong)(cus & 0xFFFFL)) << 14) | (((sqrta >> 24) & 0xFFUL) << 6);
		sbf[1][8] = ((sqrta & 0xFFFFFFUL) << 6);
		sbf[1][9] = (toe & 0xFFFFUL) << 14;

		// Subframe 3
		sbf[2][0] = 0x8B0000UL << 6;
		sbf[2][1] = 0x3UL << 8;
		sbf[2][2] = (((ulong)(cic & 0xFFFFL)) << 14) | (((ulong)((omg0 >> 24) & 0xFFL)) << 6);
		sbf[2][3] = ((ulong)(omg0 & 0xFFFFFFL)) << 6;
		sbf[2][4] = (((ulong)(cis & 0xFFFFL)) << 14) | (((ulong)((inc0 >> 24) & 0xFFL)) << 6);
		sbf[2][5] = ((ulong)(inc0 & 0xFFFFFFL)) << 6;
		sbf[2][6] = (((ulong)(crc & 0xFFFFL)) << 14) | (((ulong)((aop >> 24) & 0xFFL)) << 6);
		sbf[2][7] = ((ulong)(aop & 0xFFFFFFL)) << 6;
		sbf[2][8] = ((ulong)(omgdot & 0xFFFFFFL)) << 6;
		sbf[2][9] = (((ulong)(iode & 0xFFUL)) << 22) | (((ulong)(idot & 0x3FFFL)) << 8);

		// Subframe 4
		if (ionoutc.Vflg)
		{
			sbf[3][0] = 0x8B0000UL << 6;
			sbf[3][1] = 0x4UL << 8;
			sbf[3][2] = (dataId << 28) | (sbf4p18 << 22)
					  | (((ulong)(alpha0 & 0xFFL)) << 14) | (((ulong)(alpha1 & 0xFFL)) << 6);
			sbf[3][3] = (((ulong)(alpha2 & 0xFFL)) << 22) | (((ulong)(alpha3 & 0xFFL)) << 14)
					  | (((ulong)(beta0 & 0xFFL)) << 6);
			sbf[3][4] = (((ulong)(beta1 & 0xFFL)) << 22) | (((ulong)(beta2 & 0xFFL)) << 14)
					  | (((ulong)(beta3 & 0xFFL)) << 6);
			sbf[3][5] = ((ulong)(A1 & 0xFFFFFFL)) << 6;
			sbf[3][6] = (((ulong)((A0 >> 8) & 0xFFFFFFL)) << 6);
			sbf[3][7] = (((ulong)(A0 & 0xFFL)) << 22) | ((tot & 0xFFUL) << 14)
					  | ((wnt & 0xFFUL) << 6);
			sbf[3][8] = (((ulong)(dtls & 0xFFL)) << 22) | ((wnlsf & 0xFFUL) << 14)
					  | ((dn & 0xFFUL) << 6);
			sbf[3][9] = ((dtlsf & 0xFFUL) << 22);
		}
		else
		{
			sbf[3][0] = 0x8B0000UL << 6;
			sbf[3][1] = 0x4UL << 8;
			sbf[3][2] = (dataId << 28) | (sbf4p25 << 22);
			for (int i = 3; i < 10; i++) sbf[3][i] = 0UL;
		}

		// Subframe 5
		sbf[4][0] = 0x8B0000UL << 6;
		sbf[4][1] = 0x5UL << 8;
		sbf[4][2] = (dataId << 28) | (sbf5p25 << 22) | ((toa & 0xFFUL) << 14) | ((wna & 0xFFUL) << 6);
		for (int i = 3; i < 10; i++) sbf[4][i] = 0UL;
	}

	// ── GPS parity checksum ──────────────────────────────────────────────

	private static ulong CountBits(ulong v)
	{
		v = ((v >> 1) & 0x55555555UL) + (v & 0x55555555UL);
		v = ((v >> 2) & 0x33333333UL) + (v & 0x33333333UL);
		v = ((v >> 4) & 0x0F0F0F0FUL) + (v & 0x0F0F0F0FUL);
		v = ((v >> 8) & 0x00FF00FFUL) + (v & 0x00FF00FFUL);
		v = ((v >> 16) & 0x0000FFFFUL) + (v & 0x0000FFFFUL);
		return v;
	}

	public static ulong ComputeChecksum(ulong source, bool nib)
	{
		ulong[] bmask = {
			0x3B1F3480UL, 0x1D8F9A40UL, 0x2EC7CD00UL,
			0x1763E680UL, 0x2BB1F340UL, 0x0B7A89C0UL
		};

		ulong d    = source & 0x3FFFFFC0UL;
		ulong D29  = (source >> 31) & 0x1UL;
		ulong D30  = (source >> 30) & 0x1UL;

		if (nib)
		{
			if ((D30 + CountBits(bmask[4] & d)) % 2 != 0) d ^= (0x1UL << 6);
			if ((D29 + CountBits(bmask[5] & d)) % 2 != 0) d ^= (0x1UL << 7);
		}

		ulong D = d;
		if (D30 != 0) D ^= 0x3FFFFFC0UL;

		D |= ((D29 + CountBits(bmask[0] & d)) % 2) << 5;
		D |= ((D30 + CountBits(bmask[1] & d)) % 2) << 4;
		D |= ((D29 + CountBits(bmask[2] & d)) % 2) << 3;
		D |= ((D30 + CountBits(bmask[3] & d)) % 2) << 2;
		D |= ((D30 + CountBits(bmask[4] & d)) % 2) << 1;
		D |= ((D29 + CountBits(bmask[5] & d)) % 2);

		return D & 0x3FFFFFFFUL;
	}

	// ── Navigation message generation ────────────────────────────────────

	public static void GenerateNavMsg(GpsTime g, ChannelState chan, bool init)
	{
		const int NDwrdSbf = ChannelState.NDwrdSbf;
		const int NSbf     = ChannelState.NSbf;
		const int NDwrd    = ChannelState.NDwrd;

		var g0 = new GpsTime
		{
			Week = g.Week,
			Sec  = Math.Round(g.Sec * (1.0 / 30.0)) * 30.0  // align to 30-second frame
		};
		chan.G0 = g0;

		ulong wn  = (ulong)(g0.Week % 1024);
		ulong tow = (ulong)(g0.Sec) / 6UL;

		if (init)
		{
			// Initialize with subframe 5
			ulong prevwrd = 0UL;
			for (int iwrd = 0; iwrd < NDwrdSbf; iwrd++)
			{
				ulong sbfwrd = chan.Sbf[4][iwrd];
				if (iwrd == 1) sbfwrd |= ((tow & 0x1FFFFUL) << 13);
				sbfwrd |= (prevwrd << 30) & 0xC0000000UL;
				bool nibFlag = (iwrd == 1 || iwrd == 9);
				chan.Dwrd[iwrd] = ComputeChecksum(sbfwrd, nibFlag);
				prevwrd = chan.Dwrd[iwrd];
			}
		}
		else
		{
			// Carry over last subframe 5 words
			ulong prevwrd = 0UL;
			for (int iwrd = 0; iwrd < NDwrdSbf; iwrd++)
			{
				chan.Dwrd[iwrd] = chan.Dwrd[NSbf * NDwrdSbf + iwrd];
				prevwrd = chan.Dwrd[iwrd];
			}
		}

		for (int isbf = 0; isbf < NSbf; isbf++)
		{
			tow++;
			ulong prevwrd = chan.Dwrd[isbf * NDwrdSbf + (NDwrdSbf - 1)];
			for (int iwrd = 0; iwrd < NDwrdSbf; iwrd++)
			{
				ulong sbfwrd = chan.Sbf[isbf][iwrd];
				if (isbf == 0 && iwrd == 2) sbfwrd |= ((wn & 0x3FFUL) << 20);
				if (iwrd == 1) sbfwrd |= ((tow & 0x1FFFFUL) << 13);
				sbfwrd |= (prevwrd << 30) & 0xC0000000UL;
				bool nibFlag = (iwrd == 1 || iwrd == 9);
				chan.Dwrd[(isbf + 1) * NDwrdSbf + iwrd] = ComputeChecksum(sbfwrd, nibFlag);
				prevwrd = chan.Dwrd[(isbf + 1) * NDwrdSbf + iwrd];
			}
		}
	}
}
