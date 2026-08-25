# GPSSimulator

A .NET MAUI Blazor desktop application that generates a live GPS L1 C/A baseband
signal and transmits it through a **HackRF One** SDR, replaying a recorded or
planned route so that a connected GPS receiver reports the simulated position,
speed and heading.

The signal generation is implemented natively in C# (no external
`gps-sdr-sim.exe` process is required) and streams IQ samples continuously to the
HackRF while the route plays back.

> **Legal notice**
> Transmitting GPS signals over the air is illegal in most jurisdictions.
> Always use a **direct RF cable connection with appropriate attenuation** (or a
> shielded enclosure) between the HackRF and the receiver under test.

---

## Requirements

| Item | Notes |
|------|-------|
| Windows 10 (1809) or later | Transmission is Windows-only; other MAUI targets build but cannot transmit. |
| .NET 10 SDK | The project multi-targets Android/iOS/MacCatalyst/Windows. |
| HackRF One / PRO | Connected over USB, with a coax/attenuator path to the receiver. |
| HackRF drivers + `hackrf.dll` | The native library is loaded by the bundled `nethackrf` wrapper. |
| Internet access | Used to download broadcast ephemeris (RINEX) from NASA CDDIS. |
| `curl.exe` | Ships with Windows; used for the FTPS download from CDDIS. |

---

## Getting the application

### Prebuilt binaries

Every commit to `master` is built by GitHub Actions and published as a release.

1. Go to the [Releases page](https://github.com/viamatt/GPSSimulator/releases).
2. Download `GPSSimulator-win-x64.zip` from the latest release.
3. Extract it anywhere and run `GPSSimulator.exe`.

The build is **self-contained**, so no .NET runtime installation is required —
only 64-bit Windows 10 (1809) or later, plus HackRF USB drivers. The same zip is
also attached to each workflow run under the **Actions** tab as a build
artifact, which is useful for branches that have no release.

The binaries are unsigned, so SmartScreen may warn on first launch
(*More info → Run anyway*).

### Building from source

```sh
git clone https://github.com/viamatt/GPSSimulator.git
cd GPSSimulator

# One-time: install the MAUI Windows workload
dotnet workload install maui-windows

# Run a debug build
dotnet build GPSSimulator/GPSSimulator.csproj -f net10.0-windows10.0.19041.0
dotnet run --project GPSSimulator/GPSSimulator.csproj -f net10.0-windows10.0.19041.0
```

To produce the same self-contained package the CI workflow does:

```sh
dotnet publish GPSSimulator/GPSSimulator.csproj -c Release ^
  -f net10.0-windows10.0.19041.0 -r win-x64 --self-contained true ^
  -p:WindowsPackageType=None -p:WindowsAppSDKSelfContained=true -o publish
```

Notes:

* The project multi-targets Android/iOS/MacCatalyst/Windows; always pass
  `-f net10.0-windows10.0.19041.0` so only the Windows target — the only one
  that can transmit — is built.
* Building from Visual Studio 2022 17.12+ or Visual Studio 2026 works directly:
  open `GPSSimulator.slnx`, select the **Windows Machine** target and run.
* A `NuGet.config` at the repository root clears inherited feeds and restores
  from nuget.org only, so restore is not affected by machine-wide private feeds.
* The native HackRF DLLs are committed to the repository and copied to the
  output directory automatically.

---

## Quick start

1. Connect the HackRF
2. Launch the app.
3. In **Ephemeris (RINEX)**, click **Download Latest**. The app fetches the most
   recent broadcast navigation file and verifies it covers the current time.
4. In **Route Replay (Trip JSON / GPX)**, click **Browse** and choose a route
   file. Its summary (point count, duration, format) is displayed once loaded.
5. Optionally adjust **Playback Speed** (see below).
6. Click **Start**. The map shows the route and the live simulated position; the
   log pane shows transmission telemetry.
7. Click **Stop** to end transmission.

---

## Route file formats

### Trip JSON (`.json`)

The native format. Every position carries an `occurredAt` timestamp, latitude,
longitude, altitude, speed and heading, typically logged at ~1 Hz. Timings are
taken directly from the file, so replay is a faithful reproduction of the
original trip.

Positions with unparseable timestamps are skipped; the remainder are sorted by
time and normalised to an offset from the first point.

### GPX (`.gpx`)

GPX 1.0 and 1.1 are parsed with the [FrozenNorth.Gpx](https://www.nuget.org/packages/FrozenNorth.Gpx)
library, which handles both schema versions and namespace variations.

**Track structure is preserved.** A GPX file is hierarchical: it may contain
multiple `<trk>` tracks, each split into one or more `<trkseg>` segments, plus
separate `<rte>` routes and standalone `<wpt>` waypoints. The parser reads these
in priority order and does **not** mix them:

1. all `<trkseg>` segments of all `<trk>` tracks, in file order;
2. otherwise, `<rte>` routes;
3. otherwise, standalone `<wpt>` waypoints.

This matters because many exports contain both a detailed track *and* a couple of
start/finish waypoints. Reading them together lets those two distant waypoints
dominate the geometry and produces a straight line from start to end instead of
the recorded route. Points are sorted within each segment (segments themselves
ordered by their first timestamp) rather than in one global sort, which would
otherwise interleave separate segments into a zig-zag.

**Segment joins.** Where one segment ends more than 250 m from where the next
begins — typically a paused recording or a lifted GPS — the parser allots a fixed
5-second crossing and slides all subsequent timings. Without this, a pause either
teleports the position (synthesised timings) or crawls across the gap for the
whole pause duration (recorded timings). The route summary reports the segment
count, e.g. *GPX (3 segments)*.

GPX also differs from the trip JSON format in three ways the parser compensates
for:

**1. Timestamps are optional.**
Routes exported from planning tools (Komoot, RideWithGPS, Garmin BaseCamp,
etc.) usually contain no `<time>` elements at all. When timestamps are missing
— or only present on some points, or non-monotonic — the app synthesises timings
by walking the track at a constant assumed ground speed of **50 km/h**. The
loaded-route summary makes this explicit with
*"(timings estimated - no timestamps in file)"*. When every point *is* timed and
the sequence advances, the recorded times are used as-is.

**2. Speed and heading are not stored.**
Both are derived from the geometry: speed from great-circle distance divided by
segment time, and heading as the initial bearing towards the next point.

**3. Point density is much lower.**
A GPX route may have only a handful of points per kilometre versus 1 Hz logging.
The parser **densifies** the track, inserting interpolated points so that no two
consecutive points are more than 25 m apart. This keeps the simulated motion
smooth, keeps UI telemetry updating, and prevents large instantaneous position
jumps that a receiver would reject.

Densification only ever interpolates *within* a segment, never across a segment
boundary.

Because corners are cut between sparse points, a GPX route follows a slightly
straighter path than the original recording. Denser GPX files reproduce the
intended track more accurately.

---

## Playback speed

The **Playback Speed** slider (0.5x – 20x, with 1x/2x/5x/10x presets) scales the
route timeline only:

* At **1x** the route replays in real time.
* At **5x** a 50-minute route completes in 10 minutes, and the receiver observes
  a vehicle travelling five times faster.

**GPS time always advances at 1x** regardless of this setting. Only the position
along the route is advanced faster — nav-message timing, satellite geometry and
ephemeris validity remain correct. The estimated wall-clock replay duration is
shown beneath the slider.

This is most useful for GPX routes with synthesised timings, where the assumed
50 km/h would otherwise make a long route tedious to replay.

**Caution:** most GPS receivers refuse to track above roughly **500 m/s**
(the COCOM export-control limit). A route with a real average of 100 km/h played
at 20x implies 2000 km/h and will likely cause loss of fix. The UI shows a
warning above 10x.

---

## Ephemeris (RINEX)

The simulator needs broadcast ephemeris covering the simulated time in order to
produce a valid navigation message.

* **Download Latest** fetches from NASA CDDIS over explicit FTPS using
  `curl.exe`. **Hourly** navigation files are tried first because the current
  day's daily `brdc` file is still being written and often lags real time; daily
  files are used as a fallback.
* **Use current time** (enabled by default) sets the simulation epoch to
  `DateTime.UtcNow` rather than the first epoch in the RINEX file. This is what
  you normally want, as receivers reject signals far from their expected time.
  If coverage is stale, the app automatically re-downloads.
* A coverage indicator shows the file's valid time span and whether it covers
  now.

---

## Signal parameters

| Setting | Default | Notes |
|---------|---------|-------|
| Frequency | 1575.42 MHz | GPS L1. |
| Sample rate | 2.6 MHz | Baseband filter is set to 2x sample rate. |
| Elevation mask | 10° | With hysteresis, so satellites near the mask do not flap in and out. |
| Max satellites | 8 | Fewer channels give each satellite a larger share of the SC08 dynamic range, improving acquisition. Four are required for a fix. |
| TX VGA gain | adjustable | Prefer raising analogue gain over digital normalisation, which degrades tracking. |

Sample format is signed 8-bit IQ (SC08). Positions are sampled by the signal
engine on its own 0.1 s epoch clock rather than a wall-clock timer, so
pseudorange rate — and therefore Doppler and the receiver's reported speed —
reflects true vehicle velocity without timer jitter.

---

## Project layout

```
GPSSimulator/
├─ Components/Pages/Home.razor        Main UI: RINEX, route, controls, map, log
├─ GpsEngine/
│  ├─ GpsSignalEngine.cs              L1 C/A generation, channels, nav message, IQ output
│  ├─ TripModels.cs                   TripPoint / Trip route model and interpolation
│  ├─ JsonTripParser.cs               Trip JSON reader
│  ├─ GpxParser.cs                    GPX reader: segments, timing synthesis, densification
│  └─ RouteLoader.cs                  Format dispatch by file extension
├─ Services/
│  ├─ GpsSimulatorService.cs          Orchestration: engine, trip driver, HackRF stream
│  └─ RinexDownloadService.cs         CDDIS hourly/daily ephemeris download and caching
├─ nethackrf/                         HackRF P/Invoke wrapper and streaming API
└─ wwwroot/                           Static assets, Leaflet map interop
```

---

## External libraries and credits

### NuGet packages

| Package | Purpose |
|---------|---------|
| `Microsoft.Maui.Controls` | .NET MAUI application framework. |
| `Microsoft.AspNetCore.Components.WebView.Maui` | Hosts the Blazor UI inside the MAUI shell. |
| `Microsoft.Extensions.Logging.Debug` | Debug-output logging provider. |
| [`FrozenNorth.Gpx`](https://www.nuget.org/packages/FrozenNorth.Gpx) | GPX 1.0/1.1 reading, preserving tracks, segments, routes and waypoints. |

### Bundled source

| Component | Purpose |
|-----------|---------|
| `nethackrf` | C# P/Invoke wrapper and streaming API over `libhackrf`. Included as source under `nethackrf/`. |

### Native dependencies (Windows)

| Library | Purpose |
|---------|---------|
| `hackrf.dll` (libhackrf) | HackRF device control and USB transfer, from the [Great Scott Gadgets HackRF](https://github.com/greatscottgadgets/hackrf) project. |
| `libusb-1.0.dll` | USB transport used by libhackrf. |
| `pthreadVC2.dll` / `libwinpthread-1.dll` | Threading support required by the libhackrf Windows build. |

### Client-side

| Library | Purpose |
|---------|---------|
| [Leaflet 1.9.4](https://leafletjs.com/) | Interactive map rendering of the route and live position (loaded from unpkg CDN). |
| Bootstrap 5 | UI layout and controls (bundled under `wwwroot/lib`). |
| OpenStreetMap tiles | Map imagery, subject to the [OSM tile usage policy](https://operations.osmfoundation.org/policies/tiles/). |

### Data sources

| Source | Purpose |
|--------|---------|
| [NASA CDDIS](https://cddis.nasa.gov/) | Broadcast GPS ephemeris (RINEX navigation files). |

### Reference implementations

The signal generation and streaming design draw on:

* **[gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim) by Takuji Ebinuma** —
  the primary source for much of the simulation code in this project. The C#
  implementation of RINEX ephemeris parsing, satellite orbit and clock
  computation, ionospheric/tropospheric corrections, C/A code generation,
  navigation-message subframe assembly and range/Doppler modelling all follow
  the algorithms and structure of gps-sdr-sim, ported to managed code and
  reworked for continuous real-time streaming instead of file output.
* [multi-sdr-gps-sim](https://github.com/Mictronics/multi-sdr-gps-sim) by
  Mictronics — the reference for continuous-streaming SDR output design.

---

## Troubleshooting

**"RINEX does not cover current time"**
The daily `brdc` file for today may still be incomplete. Click **Download
Latest** again — the app prefers hourly files, which track real time closely.

**No fix, or satellites visible but not "in use"**
Check cabling and attenuation, allow a cold receiver 30–60 seconds to acquire,
and confirm the ephemeris covers the simulated time. Reducing the satellite
count raises per-satellite C/N0 and improves acquisition.

**Speed reads 0 km/h while the position moves**
Verify **Playback Speed** is not extremely low, and that the loaded route
actually contains movement between points.

**Loss of fix at high playback speed**
Reduce the playback speed. Implied velocities above ~500 m/s exceed the COCOM
limit enforced by most receivers.

**Build fails with a file-in-use error**
Close the running application before rebuilding; it locks its output binaries.

**Restore fails with a 401 from an unexpected feed**
A machine-wide `NuGet.config` may add a private feed. The repository's own
`NuGet.config` clears inherited sources; if you build outside the repository
root, pass `-s https://api.nuget.org/v3/index.json` to `dotnet restore`.

**App does not exit / hangs on close**
Closing the window while a replay is transmitting is handled automatically: the
app cancels the transmission and disposes the HackRF before shutting down. If it
still fails to exit, the device may be wedged — unplug and reconnect the HackRF.
