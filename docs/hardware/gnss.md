# GPS / GNSS Receivers

GPS handling is in `main/gnss.go`. Stratux can autodetect a receiver via udev symlinks or use
a manually configured serial device. Most receivers are **u-blox**-based, but a SiRF receiver,
a Raspberry Pi UART GPS, network GPS, and serial trackers are also supported.

## Autodetected receivers

Detection prefers the stable udev symlinks created by `debian/10-stratux.rules`:

| Receiver / chip | Symlink | Internal type | Notes |
|---|---|---|---|
| u-blox 9 | `/dev/ublox9` | `GPS_TYPE_UBX9` | |
| u-blox 8 (RY83xAI, GPYes 2.0) | `/dev/ublox8` | `GPS_TYPE_UBX8` | 80 ms PPS offset |
| u-blox 7 (VK-172, VK-162 rev 2, GPYes, RY725AI) | `/dev/ublox7` | `GPS_TYPE_UBX6or7` | |
| u-blox 6 (VK-162 rev 1) | `/dev/ublox6` | `GPS_TYPE_UBX6or7` | |
| SiRFstar IV — **BU-353-S4** (Prolific PL2303) | `/dev/prolific0` | `GPS_TYPE_PROLIFIC` | |
| Generic u-blox on the Pi UART (GPIO 8/10) | `/dev/serial0` | `GPS_TYPE_UBX_GEN` | |
| Serial-in tracker (OGN-Tracker / SoftRF) | `/dev/serialin` | `GPS_TYPE_SERIAL` | see [ogn-ais-receivers.md](ogn-ais-receivers.md) |
| SoftRF dongle (TTGO dongle, STM VCP) | `/dev/softrf_dongle` | `GPS_TYPE_SOFTRF_DONGLE` | |

GPS can also come from the **network** (`GPS_TYPE_NETWORK`) rather than a local device — e.g.
NMEA over TCP from an external source.

## Manual configuration

Set `GpsManualConfig: true` and the manual fields (see
[settings-reference.md](../settings-reference.md)):

- `GpsManualDevice` — serial path (default `/dev/ttyAMA0`)
- `GpsManualChip` — `ublox6`, `ublox7`, `ublox8`, `ublox9`, `ublox10`, or `ublox` (generic).
  Any other/empty value leaves the chip unconfigured (`GPS_TYPE_ANY`, no reconfig).
- `GpsManualTargetBaud` — target baud after reconfiguration (default `115200`)

## Baud detection & autoconfiguration

`detectOpenSerialPort()` probes candidate baud rates, validates the NMEA checksum, and keeps
the working rate:

- Auto-detect first tries `9600`.
- Manual/serial tries `115200, 38400, 9600, 230400, 500000, 1000000, 2000000`.
- SiRF tries `4800, 38400, 9600`.

For u-blox chips, Stratux sends `CFG-GNSS` to enable constellations (GPS, SBAS, Galileo,
BeiDou, QZSS, GLONASS) and sets an **airborne <2g dynamic model**, NMEA 4.0 output, and the
desired nav rate (1/2/5/10 Hz). Per-chip routines exist for u-blox 8 and 9, with a generic
path for others. The GPS time-pulse LED is disabled via UBX-CFG-TP5.

The detected receiver type is reported as `GPS_detected_type` in `/getStatus`.
