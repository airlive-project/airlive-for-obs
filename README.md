<div align="center">
  <img src=".github/logo.png" width="130" alt="Airlive for OBS">
  <h1>Airlive for OBS</h1>
  <p><b>Bring an Airlive Camera (iPhone) feed straight into OBS Studio — no separate receiver app.</b></p>

  <a href="LICENSE"><img src="https://img.shields.io/badge/License-GPLv2-blue.svg" alt="License: GPL v2"></a>
  <img src="https://img.shields.io/badge/macOS-%E2%80%A2%20Windows-lightgrey" alt="macOS / Windows">
  <img src="https://img.shields.io/badge/OBS-plugin-302E31" alt="OBS plugin">
  <a href="https://github.com/airlive-project/airlive-for-obs/releases/latest"><img src="https://img.shields.io/badge/Download-latest%20pkg-brightgreen" alt="Download latest pkg"></a>
</div>

---

An **OBS Studio source plugin** that brings a live video feed from the **Airlive
Camera** iOS app directly into OBS as an async video source — no separate macOS
"Airlive Studio" receiver needed. Use an iPhone as a clean, low-latency camera
straight inside OBS.

> Plugin binary: `airlive-obs`. OBS source / module name: **"Airlive OBS"**.
> Source type id: `airlive_obs_source`. Named distinctly so it is never confused
> with any older Airlive screen-mirroring plugin.

The iPhone encodes a display-ready **H.264 1080p** proxy of its viewfinder and
pushes it over plain **TCP**. This plugin is the **server/receiver**: it listens
on an ephemeral port, advertises itself via Bonjour so the iPhone can discover
it, accepts one iPhone per source, parses the framing, decodes H.264 with
FFmpeg, and outputs frames to OBS.

## How it works

```
 iPhone (TCP client)                         OBS plugin (TCP server, this repo)
 ───────────────────                         ──────────────────────────────────
 browse _airlive._tcp  ───── Bonjour ─────►  one advert per OBS source
 connect out to port   ───── TCP ─────────►  accept() on OS-assigned port
 ARLV framed packets   ─────────────────►    PacketParser (stateful)
   type 0 SPS/PPS                              └► H264Decoder.setParameterSets
   type 1 AVCC sample                          └► H264Decoder.decodeSample
   type 2 control (ignored in v1)                 └► AVFrame
                                                      └► obs_source_output_video
```

- **Discovery / roles** — the iPhone is the client and connects *out*; we are
  the server. We bind port **0** so the OS picks a free port (never a hardcoded
  7777 — many receivers coexist on one LAN), then advertise **one
  `_airlive._tcp` instance per OBS source** with a TXT record:
  `v=1, role=obs, did=<per-install UUID>, dev=<group name>, sid=<source id>,
  src=<source name>, busy=0|1`. The phone groups instances by `did` (header =
  `dev`), lists each `src`, and flips the row to "In use" on `busy=1`.
- **One connection per source.** `busy` flips on connect/disconnect; on
  disconnect we keep listening so the phone can reconnect.
- **No cap** on the number of sources — add as many as you want.

## Source layout

| File | Responsibility |
|------|----------------|
| `src/wire.hpp` | Wire-format constants (magic, header size, limits, NAL types) |
| `src/packet-parser.{hpp,cpp}` | Stateful 18-byte-header framing parser; resyncs on bad headers |
| `src/h264-decoder.{hpp,cpp}` | FFmpeg H.264 decode; AVCC→Annex-B, SPS/PPS caching |
| `src/bonjour-service.{hpp,cpp}` | `_airlive._tcp` advertise + TXT record (dns_sd) |
| `src/net-compat.hpp` | POSIX/Winsock socket shim |
| `src/airlive-connection.{hpp,cpp}` | Worker thread: listen → accept → parse → decode |
| `src/airlive-source.cpp` | `obs_source_info`, properties, AVFrame→`obs_source_frame`, module entry |

## Build

Requires the **OBS Studio dev files** (`libobs`) and **FFmpeg**
(`libavcodec`/`libavutil`/`libswscale`), plus a Bonjour/mDNS backend.

```bash
cmake -B build -G Ninja
cmake --build build
```

- **macOS** — dns_sd is built in. Install FFmpeg (`brew install ffmpeg`) and OBS
  dev files.
- **Linux** — `libavahi-compat-libdnssd-dev` provides the dns_sd API over Avahi.
- **Windows** — install the Bonjour SDK (provides `dns_sd.h` + `dnssd.lib`).

For a production multi-platform build with signing/packaging/CI, drop these
`src/*.cpp` files into [obs-plugintemplate](https://github.com/obsproject/obs-plugintemplate)
and use its CMake. The bundled `CMakeLists.txt` here is a pragmatic standalone
build for local iteration.

Copy the built module and `data/` into your OBS `plugins` directory, then add an
**"Airlive Camera (iPhone)"** source in OBS.

## Verified against OBS source

The decode + output path mirrors OBS's own production FFmpeg camera decoder
([`plugins/win-dshow/ffmpeg-decode.c`](https://github.com/obsproject/obs-studio/blob/master/plugins/win-dshow/ffmpeg-decode.c)):
same `avcodec_alloc_context3`/`open2` init, plane pass-through (no copy),
`AV_PKT_FLAG_KEY` on keyframes, OBS-accurate colorspace/TRC mapping, and
`obs_source_frame2` + `obs_source_output_video2` with explicit `range`/`trc`
(the legacy v1 output path does not honour partial range, and H.264 is
limited-range). The pixel-format mapping matches
[`obs-ffmpeg-formats.h`](https://github.com/obsproject/obs-studio/blob/master/plugins/obs-ffmpeg/obs-ffmpeg-formats.h),
and the CMake/module boilerplate matches
[`obs-plugintemplate`](https://github.com/obsproject/obs-plugintemplate)
(`find_package(libobs)` → `OBS::libobs`). The AVCC→Annex-B conversion is ours
because the Airlive wire is AVCC + separate SPS/PPS, whereas win-dshow's
cameras already deliver Annex-B.

## Tested

The two error-prone subsystems were validated against the real toolchain:

- **Framing parser** — unit-tested for multi-packet reads, byte-by-byte
  reassembly, garbage resync, bad-version recovery, oversize-length rejection,
  and control packets.
- **Decode path** — driven end-to-end with a real H.264 clip repackaged into the
  Airlive wire payloads (SPS/PPS from `avcC`, AVCC samples): 30 samples → 30
  decoded frames at the correct resolution.

To test against hardware: run Airlive Camera on an iPhone on the same LAN, point
it at the advertised source, and watch frames land in OBS.

## Source properties

Each "Airlive OBS" source exposes:

- **Status** — read-only: connected / waiting, live resolution+fps (from the
  decoded stream), and the iPhone's device / camera resolution / colour space /
  lens (from its control-state snapshot). A **Refresh status** button re-reads it.
- **Fixed delay (ms)** — a deliberate presentation buffer (0–1000 ms, default
  **120**), Studio-style. Decoded frames are held in an FFmpeg ref-counted queue
  and released by a presenter thread after the delay elapses; the pixel mapping
  runs on that thread, so there's no contention with the socket/decoder. Set to
  0 for lowest latency.
- **Device name / Source name** — the Bonjour `dev` (group header on the iPhone)
  and `src` (row label) labels.

## Tally (Studio Mode)

When this source goes to **Program**, the plugin sends `setCue:"program"` back
to the iPhone over the control channel; in **Preview** it sends `"preview"`;
otherwise `"none"`. The operator behind the camera sees the live/staged stripe
in the Airlive Camera viewfinder. Cue is re-sent whenever the phone (re)connects
so it learns its current state immediately. This is the only thing the plugin
*writes* back to the phone in this version.

## Scope

- **v1 is video-only.** The protocol defines no audio packet type.
- **H.264 only** by design — the wire is a fixed display-ready proxy. The camera
  can *record* HEVC/ProRes locally, but never sends them over the wire.
- **Control channel (type 2)** — partially used: inbound state snapshots feed
  the status display, and tally (`setCue`) is sent outbound. Full remote camera
  control (ISO/shutter/WB/lens/zoom/focus/fps/LUT) is the remaining phase; wire
  priority is **tally ▸ video ▸ settings**, and tally is already on the
  latency-critical path.

## Install (macOS)

Grab the signed installer from
[Releases](https://github.com/airlive-project/airlive-for-obs/releases/latest). It
installs into your **user** OBS plugins folder (`~/Library/Application Support/
obs-studio/plugins/`) — no admin — and ships an `uninstall.command` that removes
it cleanly. Restart OBS, then add **+ → Airlive Camera** (direct iPhone) or
**+ → Airlive Bridge** (relay from the Airlive Bridge app).

## License

**GPL-2.0** — see [`LICENSE`](LICENSE) and [`THIRD-PARTY-NOTICES.md`](THIRD-PARTY-NOTICES.md),
the standard license for an OBS plugin (it links GPL-2.0 libobs). The license
covers the code; it doesn't grant use of the "Airlive" name or logo.
