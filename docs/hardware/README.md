# Supported Hardware

This section documents the hardware Stratux actually supports **in the code** — how each
device is detected, assigned, and configured. It complements the user-oriented buyer's-guide
content in the [project wiki](https://github.com/stratux/stratux/wiki/Supported-Hardware);
where the wiki says "what to buy," these docs say "how the firmware sees it" and "how to add
a new device."

| Topic | Doc |
|---|---|
| SDR dongles & band assignment (1090ES / 978 UAT / 868 OGN / AIS) | [sdr-and-bands.md](sdr-and-bands.md) |
| GPS / GNSS receivers | [gnss.md](gnss.md) |
| Barometric & IMU/AHRS sensors | [sensors.md](sensors.md) |
| OGN trackers (RX/TX), AIS, and external ADS-B receivers (Ping/Pong/UATRadio) | [ogn-ais-receivers.md](ogn-ais-receivers.md) |
| Cooling fan | see `fancontrol_main/` (PID PWM fan, default BCM pin 18, own systemd service) |

## How devices are recognized

Two complementary mechanisms:

1. **udev rules** (`debian/*.rules`) match USB vendor/product IDs (and sometimes the product
   string) and create stable `/dev/...` symlinks. This is the canonical list of "officially
   recognized" devices — Stratux opens the symlinks, not raw `/dev/ttyUSB*`.
2. **Runtime probing** in the Go daemon: SDR dongles are tagged and assigned by their EEPROM
   serial prefix (`main/sdr.go`); GPS chips are autodetected by baud/NMEA probing
   (`main/gnss.go`); baro/IMU chips are probed by I²C WHO_AM_I (`main/sensors.go`).

## udev-recognized USB devices

From `debian/10-stratux.rules`, `debian/99-uavionix.rules`, and `debian/99-pong.rules`:

| Device | VID:PID (or match) | Symlink |
|---|---|---|
| u-blox 9 GNSS | `1546:01a9` | `/dev/ublox9` |
| u-blox 8 GNSS (RY835/836AI, GPYes 2.0) | `1546:01a8` | `/dev/ublox8` |
| u-blox 7 GNSS (VK-172, RY725AI, GPYes) | `1546:01a7` | `/dev/ublox7` |
| u-blox 6 GNSS (VK-162) | `1546:01a6` | `/dev/ublox6` |
| Stratux UATRadio (low-power 978 UAT) | `0403:7028` (FTDI) | `/dev/uatradio` |
| Prolific PL2303 (BU-353-S4 GPS, TU-S9 serial) | `067b:2303` | `/dev/prolific*` |
| CP210x "Stratux Serialout" | interface-string match | `/dev/serialout0` |
| CP210x "Stratux Serialout NMEA" | interface-string match | `/dev/serialout_nmea0` |
| SC16IS752 I²C UART (ports 0/1) | i2c subsystem | `/dev/serialout_nmea1`, `…2` |
| SoftRF Standalone (NodeMCU/ESP32 + CP2102) | `10c4:ea60` "DIY SoftRF" | `/dev/serialin` |
| TTGO T-Beam (ESP32 + CP2104) | `10c4:ea60` "CP2104 USB to UART Bridge" | `/dev/serialin` |
| TTGO T-Beam (CH9102/QinHeng) | `1a86:55d4` | `/dev/serialin` |
| TTGO T-Beam S3 Supreme (native ESP32 USB) | `303a:1001`, `303a:8133` | `/dev/serialin` |
| TTGO T-Echo (nRF52) | `239a:8029` | `/dev/serialin` |
| TTGO dongle edition (STM32 VCP) | `0483:5740` | `/dev/softrf_dongle` |
| uAvionix Ping (MavLink / Raw) | `0403:74f0`, `0403:74f1` | `/dev/ping` |
| uAvionix PingUSB (MavLink) | `0403:6015` | `/dev/pingusb` |
| pongradio Pong dual ADS-B | `0403:6998` (FTDI) | `/dev/pong` |

> **Adding new hardware:** add a udev rule for the stable symlink (if USB-serial) and the
> detection/handling branch in the relevant Go file (`sdr.go`, `gnss.go`, `sensors.go`,
> `ping.go`/`pong.go`, or `tracker.go`). Per [building.md](../building.md), adding hardware
> support is a PR-sized change rather than a direct push.

## Hardware Stratux does *not* drive

There is no dedicated RTC, status-LED, or physical-button driver in the codebase. (The GPS
time-pulse LED is actively disabled via UBX-CFG-TP5.) Power management on T-Beam trackers is
handled by the tracker firmware, not the Pi.
