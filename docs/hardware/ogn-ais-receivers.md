# OGN Trackers, AIS, and External ADS-B Receivers

This page covers the 868 MHz OGN/FLARM stack (receive **and** transmit), AIS, and the
external USB ADS-B receivers (uAvionix Ping, Pong, and the Stratux UATRadio).

## OGN / FLARM (868 MHz)

### Receive

An RTL-SDR tagged `stratux:868` (see [sdr-and-bands.md](sdr-and-bands.md)) feeds the
`ogn-rx-eu` binary (prebuilt for arm/aarch64/x86 in `ogn/`, launched from `main/sdr.go`). It
decodes multiple 868 MHz protocols — OGN, FLARM, PilotAware (PAW), FANET, and ADS-L. APRS
parsing is in `main/ogn-aprs.go` (protocol prefixes `ICA`, `FLR`, `SKY`, `PAW`, `OGN`, `RND`,
`FMT`, `MTK`, `XCG`, `FAN`, `FNT`). Stratux can additionally pull traffic from the OGN
internet feed (`aprs.glidernet.org:14580`). The OGN device database (`ogn/ddb.json`) maps
device IDs to tail numbers. Status fields: `OGN_connected`, `OGN_messages_*`, `OGN_noise_db`,
`OGN_gain_db`. Enabled by `OGN_Enabled` (and `APRS_Enabled` for the APRS path).

### Transmit (ownship broadcast)

Stratux can transmit OGN/FLARM in two ways:

1. **I²C TX module** (RFM95 / SX1276). When `OGNI2CTXEnabled` is set, `ogn-rx-eu` is launched
   with transmit enabled (otherwise `-t off` is passed). Reported as `OGN_tx_enabled`.
2. **An attached serial tracker** (below) that does its own RF transmit.

The ownship identity broadcast is configured via the `OGN*` settings — `OGNAddr`,
`OGNAddrType` (`0`=random, `1`=ICAO, `2`=FLARM, `3`=OGN), `OGNAcftType`, `OGNPilot`,
`OGNReg`, `OGNTxPower` (see [settings-reference.md](../settings-reference.md)). In random
("stealth") mode the address changes periodically; `OGNPrevRandomAddr` retains the previous
ID for filtering.

## External serial trackers

These connect over USB and present as `/dev/serialin` (autodetected at 115200/38400 baud).
They provide GPS NMEA (and often baro), and Stratux pushes the ownship config (aircraft type,
address, pilot, TX power) back to them. Driver dispatch is in `main/gnss.go`; per-tracker
parsers are in `main/tracker.go`:

| Tracker | Detected by | GPS type |
|---|---|---|
| **OGN-Tracker** (TTGO T-Beam, ESP32) | `$POGNR` / `$POGNS` | `GPS_TYPE_OGNTRACKER` |
| **GXAirCom** | `$PFLAV,…,GXAircom` / `$PGXCF` / `LK8EX1` | `GPS_TYPE_GXAIRCOM` |
| **SoftRF** | `$PSRFH` / `$PSRFS` | `GPS_TYPE_SOFTRF` |

Baro from a tracker is taken from `$PGRMZ` (`BARO_TYPE_NMEA`) or the OGN-Tracker's own baro
(`BARO_TYPE_OGNTRACKER`).

### OGN-Tracker firmware shipped with Stratux

The repo ships OGN-Tracker firmware images (`ogn/*.zip`), flashable to a TTGO T-Beam via
`ogn/install-ogntracker-firmware-pi.sh` (`esptool.py` over `/dev/serialin`). Built by
`ogn/create_stratux_fw.sh`, supported board variants are:

- TTGO **T-Beam v0.7** + SX1276 (u-blox GPS)
- TTGO **T-Beam v1.0** + SX1276 or SX1262 (AXP power, u-blox GPS)
- TTGO **T-Beam v1.2** + SX1276 or SX1262 (XPowers, u-blox GPS)
- TTGO **T-Beam S3 Supreme** (ESP32-S3) + SX1262, with MTK or u-blox GPS

Firmware features include OGN, ADS-L, FANET, BMP280/BME280 baro, an OLED display, and GPS PPS.

## AIS (marine)

An RTL-SDR tagged `stratux:162` feeds the `rtl_ais` binary (vendored in `rtl-ais/`). Stratux
reads the NMEA AIVDM/AIVDO output over TCP `127.0.0.1:10110` (`main/ais.go`) and decodes it
with `github.com/BertoldVdb/go-ais`, handling message types 1/2/3 (position), 5 (static), and
27 (long-range). Vessels are surfaced as ground targets out to ~150 km. Enabled by
`AIS_Enabled`; status fields `AIS_connected`, `AIS_messages_*`.

## External USB ADS-B receivers

These are alternatives to RTL-SDR dongles — fully decoded receivers connected over USB serial.

### uAvionix Ping (`main/ping.go`, `Ping_Enabled`)

| Model | Device | Baud | Notes |
|---|---|---|---|
| Ping (EFB / 1090ES + UAT) | `/dev/ping` | 2 Mbaud | Relays `*` (1090ES) and `+`/`-` (UAT) reports into dump1090/dump978. |
| PingUSB (MAVLink) | `/dev/pingusb` | 57600 | Parses MAVLink `ADSB_VEHICLE` (msg id 246) traffic. |
| SoftRF (fallback) | `/dev/softrf` | 38400 | Treated like a Ping device. |

udev: `debian/99-uavionix.rules`. Status: `Ping_connected`.

### Pong (`main/pong.go`, `Pong_Enabled`)

The pongradio **Pong** dual-band ADS-B receiver connects at `/dev/pong` (3 Mbaud), relaying
1090ES (`*`) and UAT (`+`/`-`). It tracks heartbeats (`Pong_Heartbeats`) and supports a
firmware update via `/tmp/update_pong.zip` (uploaded through `POST /updatePong`). It
**auto-enables** when `/dev/pong` appears. udev: `debian/99-pong.rules` (`0403:6998`). Status:
`Pong_connected`.

### Stratux UATRadio (`main/lowpower_uat.go`)

An FTDI-based low-power **978 MHz UAT** radio ("UATRadio v1.0") at `/dev/uatradio`, 2 Mbaud.
udev: `debian/10-stratux.rules` (`0403:7028`). Status: `UATRadio_connected`.

## Hardware-build branding

The presence of `/etc/FlightBox` or `/etc/Merlin` sets a hardware-build label
(`HardwareBuild` in `/getStatus`). These are branding flags for prebuilt units, not receiver
integrations.
