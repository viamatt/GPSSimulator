namespace GPSSimulator.GpsEngine;

/// <summary>GPS time: week number + seconds within week</summary>
public struct GpsTime
{
	public int Week;
	public double Sec;

	public static GpsTime Zero => new GpsTime { Week = -1, Sec = 0 };
}

/// <summary>Calendar date/time in UTC</summary>
public struct GpsDateTime
{
	public int Y, M, D, Hh, Mm;
	public double Sec;
}

/// <summary>Ionospheric and UTC correction parameters from RINEX header</summary>
public class IonoUtc
{
	public bool Enable;
	public bool Vflg;
	public double Alpha0, Alpha1, Alpha2, Alpha3;
	public double Beta0, Beta1, Beta2, Beta3;
	public double A0, A1;
	public int Dtls, Tot, Wnt;
	public int Dtlsf, Dn, Wnlsf;
}

/// <summary>GPS satellite ephemeris (ICD-GPS-200)</summary>
public class Ephemeris
{
	public bool Valid;
	public GpsDateTime T;
	public GpsTime Toc;   // Time of Clock
	public GpsTime Toe;   // Time of Ephemeris
	public int Iodc, Iode;
	public double Deltan, Cuc, Cus, Cic, Cis, Crc, Crs;
	public double Ecc, Sqrta, M0, Omg0, Inc0, Aop;
	public double Omgdot, Idot;
	public double Af0, Af1, Af2, Tgd;
	public int Svhlth, CodeL2;
	// Derived
	public double N, Sq1e2, A, Omgkdot;
}

/// <summary>Pseudorange observation for a single satellite</summary>
public struct RangeData
{
	public GpsTime G;
	public double Range;   // pseudorange (m)
	public double Rate;
	public double D;       // geometric distance
	public double[] Azel;  // [azimuth, elevation] radians
	public double IonoDelay;
}

/// <summary>Per-channel (per-satellite) simulation state</summary>
public class ChannelState
{
	public const int CaSeqLen = 1023;
	public const int NDwrdSbf = 10;
	public const int NSbf = 5;
	public const int NDwrd = (NDwrdSbf * (NSbf + 1));

	public int Prn;
	public int[] Ca = new int[CaSeqLen];
	public double FCcarr;     // Carrier freq offset (Hz)
	public double FCode;      // Code frequency (Hz)
	public uint CarrPhase;    // Carrier phase accumulator (integer)
	public int CarrPhasestep; // Phase step per sample
	public double CodePhase;  // Code phase (chips)
	public GpsTime G0;
	public ulong[][] Sbf = new ulong[NSbf][];  // 5 subframes x 10 words
	public ulong[] Dwrd = new ulong[NDwrd];    // data words
	public int Iword, Ibit, Icode;
	public int DataBit;  // +1 or -1
	public int CodeCA;   // +1 or -1
	public double[] Azel = new double[2];
	public RangeData Rho0;

	public ChannelState()
	{
		for (int i = 0; i < NSbf; i++)
			Sbf[i] = new ulong[NDwrdSbf];
		Rho0.Azel = new double[2];
	}
}
