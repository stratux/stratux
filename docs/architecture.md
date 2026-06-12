# Architecture Overview

Stratux turns a Raspberry Pi + RTL-SDR dongles into a multi-band aviation receiver
(1090 ADS-B, 978 UAT, OGN/FLARM on 868 MHz, AIS) and broadcasts fused traffic, weather, GPS,
and AHRS data to Electronic Flight Bags as GDL90 (and several other formats) over Wi-Fi. It is
a single long-running Go daemon plus a few bundled C programs, served alongside a web
configuration UI.

> This is the human-readable companion to the agent-oriented [`AGENTS.md`](../AGENTS.md) at the
> repo root. For build mechanics see [building.md](building.md); for the data interfaces see
> the [integration guide](integration/README.md) and [http-api.md](http-api.md).

## The daemon (`main/` → `stratuxrun`)

`main/gen_gdl90.go` holds `func main()` and the central shared state. Every other file in
`main/` is a feature module that `main()` wires together by launching goroutines. Three
mutex-guarded globals are the spine:

- **`globalSettings`** — user config, serialized to `stratux.conf`. Adding a field exposes it
  automatically via `/getSettings` / `/setSettings` and to the web UI. See
  [settings-reference.md](settings-reference.md).
- **`globalStatus`** — runtime status (message counts, device state, CPU temp, errors),
  exposed via `/getStatus`.
- **`mySituation`** — current GPS/AHRS fix and attitude, exposed via `/getSituation`.

## Signal sources — three C-integration patterns

The key to the codebase is that the three signal-source families integrate with C in three
different ways:

1. **978 UAT → linked C library.** `dump978/` is compiled into `libdump978.so` and bound into
   Go via cgo (`godump978`). UAT frames are decoded in-process, then parsed by `uatparse`
   (FIS-B weather, NEXRAD, …).
2. **1090ES, OGN, AIS → external subprocesses.** `main/sdr.go` spawns `dump1090` (1090ES),
   `ogn-rx-eu` (868 MHz), and `rtl_ais` (AIS) with `exec.Command`, monitors them, and
   **auto-restarts on crash**.
3. **GPS / baro / IMU → direct hardware.** `main/gnss.go` (serial GPS with chip autodetect),
   `main/sensors.go`, and the `sensors/` package (baro/IMU drivers).

SDR dongles are assigned to bands by EEPROM serial prefix (`stratux:1090`, `stratux:978`, …);
`main/sdr.go` owns assignment and reconfiguration. See
[hardware/sdr-and-bands.md](hardware/sdr-and-bands.md) and the rest of the
[hardware docs](hardware/README.md).

## Fusion and output

`main/traffic.go` is the heart: it merges traffic targets from all sources, dedupes,
extrapolates positions between updates, and estimates Mode-S target distance. Outputs:

- **GDL90** binary messages built in `gen_gdl90.go`, sent over UDP `:4000` and mirrored on the
  `/gdl90` WebSocket.
- **FLARM/NMEA** (`flarm-nmea.go`) over UDP/TCP `:2000`, serial, and BLE.
- **X-Plane / ForeFlight-sim** (`xplane.go`) over UDP `:49002`; **Cursor-on-Target** input
  (`cot-in.go`).
- Traffic/situation history to SQLite via `main/datalog.go`.

See the [integration guide](integration/README.md) for the full transport map and the
`Capability` routing bitmask.

## Web UI + HTTP/WebSocket API

`managementInterface()` in `main/managementinterface.go` is the HTTP server. It serves the web
UI and the JSON/WebSocket API ([http-api.md](http-api.md)). No auth — it is a local AP
network. The frontend (`web/`) is an **AngularJS** single-page app (mobile-angular-ui +
OpenLayers maps); each screen is a "plate" (HTML in `web/plates/*.html`, controller in
`web/plates/js/*.js`).

## Other binaries

- `fancontrol_main/` → `fancontrol` (PID-controlled cooling fan, its own systemd service).
- `common/` — code shared between `main` and `fancontrol` (CPU temp, aviation equations,
  helpers).

## Repo layout

| Path | What |
|---|---|
| `main/` | the daemon (Go) |
| `common/` | shared Go helpers |
| `dump978/`, `godump978/`, `uatparse/` | UAT: C lib, cgo wrapper, FIS-B parser |
| `dump1090/`, `rtl-ais/`, `ogn/ogn-tracker/`, `image_build/pi-gen/` | git submodules |
| `sensors/` | baro/IMU drivers |
| `web/` | AngularJS UI and assets |
| `debian/` | systemd units, udev rules, `.deb` packaging, boot/network templates |
| `image_build/` | pi-gen stages producing the Pi SD-card image |
| `test/` | standalone diagnostic tools; `test-data/` sample logs; `notes/` design notes |
| `docs/` | developer docs (this folder) |
