# Stratux Developer Documentation

These are the **developer-facing** docs that track the code (build, architecture, APIs,
hardware integration). User-facing how-tos live in the
[project wiki](https://github.com/stratux/stratux/wiki) — see
[the main README](../README.md#docs-in-repository-vs-in-wiki) for the rationale of that split.

## Start here

- **[architecture.md](architecture.md)** — what the daemon is, the three signal-source
  patterns, fusion and output, the web UI. Read this first.
- **[building.md](building.md)** — build targets, CI/release workflows, repo organization, and
  the OTA update process.
- **[dev-setup.md](dev-setup.md)** — setting up a development environment (remote on a Pi, or
  local Linux).

## Interfaces (for EFB / app / tool developers)

- **[integration/README.md](integration/README.md)** — the transport map (GDL90, FLARM/NMEA,
  X-Plane, CoT) and the `Capability` routing bitmask. Start here for output integration.
  - [integration/gdl90.md](integration/gdl90.md) — GDL90 over UDP, recognizing Stratux, sleep
    mode, traffic lifecycle, ForeFlight specifics.
  - [integration/other-transports.md](integration/other-transports.md) — FLARM/NMEA
    (TCP/UDP/serial/BLE), X-Plane, Cursor-on-Target.
- **[http-api.md](http-api.md)** — the full HTTP + WebSocket JSON API.
- **[settings-reference.md](settings-reference.md)** — every `stratux.conf` / `/getSettings`
  field.

## Hardware

- **[hardware/README.md](hardware/README.md)** — overview + the udev-recognized USB device
  table.
  - [hardware/sdr-and-bands.md](hardware/sdr-and-bands.md) — RTL-SDR dongles and band
    assignment.
  - [hardware/gps.md](hardware/gps.md) — GPS/GNSS receivers.
  - [hardware/sensors.md](hardware/sensors.md) — barometric and IMU/AHRS sensors.
  - [hardware/ogn-ais-receivers.md](hardware/ogn-ais-receivers.md) — OGN trackers (RX/TX),
    AIS, and external ADS-B receivers (Ping/Pong/UATRadio).

## Tech debt

Point-in-time audits tracking cleanup backlogs. Findings are candidates, not commitments —
re-verify before acting.

- **[tech-debt/modernization.md](tech-debt/modernization.md)** — outdated dependencies,
  deprecated language/stdlib usage, EOL frameworks (AngularJS), and aging build/CI/packaging
  tooling, with a prioritized backlog and suggested sequencing.
- **[tech-debt/dead-code.md](tech-debt/dead-code.md)** — functions, types, and fields that are
  defined but never referenced.
- **[tech-debt/deprecations.md](tech-debt/deprecations.md)** — whole features and hardware
  integrations that are candidates for outright removal (e.g. the out-of-production uAvionix
  Ping receiver), each with a complete removal inventory.

## Doc conventions

- Keep these docs in sync with the code in the same PR — they are reviewed alongside code
  changes.
- Cite code by package/file where it helps (`main/sdr.go`, `main/traffic.go`, …) rather than
  pinning line numbers that drift.
- User how-tos and buyer's-guide content belong in the wiki; cross-link instead of
  duplicating.
