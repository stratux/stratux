# Settings Reference (`stratux.conf`)

Every Stratux setting is a field on the `settings` struct in `main/gen_gdl90.go`
(`globalSettings`). The struct is serialized as-is to `stratux.conf`
(`/boot/firmware/stratux.conf` on the Pi, `~/.stratux.conf` for non-root desktop
dev) and is exposed verbatim over the HTTP API:

- `GET /getSettings` returns the whole object.
- `POST /setSettings` accepts a partial object — send only the fields you want to change.

Adding a setting in code is just adding a struct field; it then appears automatically in
the API and is readable by the web UI by field name. See [http-api.md](http-api.md) for the
endpoints and [the integration guide](integration/README.md) for the output transports that
several of these fields configure.

> Field names below are the exact JSON keys. Types are the Go types. "Advanced" marks fields
> that are not surfaced in the standard web UI and are intended to be set via `/setSettings`
> or by editing `stratux.conf` directly.

## Receivers / signal sources

| Field | Type | Description |
|---|---|---|
| `UAT_Enabled` | bool | 978 MHz UAT receiver (FIS-B weather + UAT traffic). |
| `ES_Enabled` | bool | 1090 MHz ES (ADS-B) receiver via `dump1090`. |
| `OGN_Enabled` | bool | 868 MHz OGN/FLARM receiver via `ogn-rx-eu`. |
| `APRS_Enabled` | bool | APRS traffic ingestion (glidernet/OGN path). |
| `AIS_Enabled` | bool | Marine AIS receiver via `rtl_ais`. |
| `Ping_Enabled` | bool | uAvionix Ping external USB receiver (`/dev/ping`, `/dev/pingusb`). |
| `Pong_Enabled` | bool | Pong dual-band external USB receiver (`/dev/pong`). Auto-enables when the device appears. |
| `GPS_Enabled` | bool | GPS/GNSS receiver. |
| `BMP_Sensor_Enabled` | bool | Barometric pressure sensor (BMP280/388/390). |
| `IMU_Sensor_Enabled` | bool | IMU / AHRS sensor (ICM-20948, MPU-9250 family). |

See [hardware/sdr-and-bands.md](hardware/sdr-and-bands.md),
[hardware/gnss.md](hardware/gnss.md), [hardware/sensors.md](hardware/sensors.md), and
[hardware/ogn-ais-receivers.md](hardware/ogn-ais-receivers.md) for the hardware each enables.

## Outputs / network

| Field | Type | Description |
|---|---|---|
| `NetworkOutputs` | []networkConnection | UDP outputs. Default: `:4000` GDL90, `:2000` FLARM/NMEA, `:49002` X-Plane/FF-sim. Each entry carries a `Capability` bitmask (see [other-transports.md](integration/other-transports.md)). |
| `SerialOutputs` | map[string]serialConnection | Serial output ports (e.g. EFIS over RS-232/USB). Serializes as `null` when empty. |
| `BleOutputs` | []bleConnection | Bluetooth LE outputs (FLARM/NMEA over GATT — SoftRF-style `FFE0/FFE1` and Nordic UART profiles). |
| `StaticIps` | []string | Additional client IPs to always send GDL90 to (beyond DHCP-lease clients). |
| `DisplayTrafficSource` | bool | Annotate traffic with its source (UAT/ES/OGN) for debugging. |

## Wi-Fi / networking

| Field | Type | Description |
|---|---|---|
| `WiFiCountry` | string | Regulatory country code (affects allowed channels/power). |
| `WiFiSSID` | string | Access-point SSID. |
| `WiFiChannel` | int | Access-point channel. |
| `WiFiSecurityEnabled` | bool | Enable WPA on the AP. |
| `WiFiPassphrase` | string | AP passphrase (when security enabled). |
| `WiFiMode` | int | Wi-Fi mode (access point / client / direct). |
| `WiFiDirectPin` | string | Wi-Fi Direct PIN. |
| `WiFiIPAddress` | string | Static AP IP address (default `192.168.10.1`). |
| `WiFiClientNetworks` | []wifiClientNetwork | Saved networks for client mode. |
| `WiFiInternetPassThroughEnabled` | bool | Route internet from a client-mode uplink to AP clients. |

## GPS

| Field | Type | Description |
|---|---|---|
| `GpsManualConfig` | bool | *Advanced.* Disable autodetect and use the manual fields below. |
| `GpsManualDevice` | string | *Advanced.* Serial device path. Default `/dev/ttyAMA0`. |
| `GpsManualChip` | string | *Advanced.* Chip: `ublox6`, `ublox7`, `ublox8`, `ublox9`, `ublox10`, or `ublox` (generic). Other/empty → unconfigured. |
| `GpsManualTargetBaud` | int | *Advanced.* Target baud after reconfig. Default `115200`. |

See [hardware/gnss.md](hardware/gnss.md) for autodetection and chip support.

## Sensors / AHRS

| Field | Type | Description |
|---|---|---|
| `IMUMapping` | [2]int | Maps aircraft axes to sensor axes (accelerometer). Set by the orientation wizard. |
| `SensorQuaternion` | [4]float64 | *Advanced.* AHRS calibration quaternion (sensor→aircraft frame). Set by `POST /cageAHRS`; can be set manually for aircraft-specific alignment. |
| `C` | [3]float64 | *Advanced.* IMU accelerometer zero-bias. |
| `D` | [3]float64 | *Advanced.* IMU gyro zero-bias. |
| `AltitudeOffset` | int | Barometric pressure-altitude offset. |
| `GLimits` | string | G-meter limit configuration. |

## SDR tuning

| Field | Type | Description |
|---|---|---|
| `PPM` | int | SDR clock correction in parts-per-million (fallback when not encoded in the dongle serial). |
| `Dump1090Gain` | float64 | *Advanced.* RTL gain for the 1090ES dongle. Default `37.2`. |

## Ownship / traffic

| Field | Type | Description |
|---|---|---|
| `OwnshipModeS` | string | Own aircraft Mode S/ICAO hex address — used to filter your own ADS-B returns. |
| `WatchList` | string | ICAO addresses to flag/watch. |
| `EstimateBearinglessDist` | bool | Estimate distance for bearingless targets (FLARM/Mode-S with no position). |
| `RadarLimits` | int | Radar view altitude limit. |
| `RadarRange` | int | Radar view range. |

## OGN tracker (transmit) config

Used when an OGN/FLARM transmitter is attached (I2C TX module or a serial OGN-Tracker /
GXAirCom / SoftRF). See [hardware/ogn-ais-receivers.md](hardware/ogn-ais-receivers.md).

| Field | Type | Description |
|---|---|---|
| `OGNI2CTXEnabled` | bool | Enable the I2C TX module on `ogn-rx-eu` (omits `-t off`). |
| `OGNAddr` | string | Tracker address (hex). |
| `OGNAddrType` | int | Address type: `0`=random, `1`=ICAO, `2`=FLARM, `3`=OGN. |
| `OGNAcftType` | int | Aircraft type code. |
| `OGNPilot` | string | Pilot name. |
| `OGNReg` | string | Aircraft registration. |
| `OGNTxPower` | int | Transmit power. |

## Logging / diagnostics

| Field | Type | Description |
|---|---|---|
| `DEBUG` | bool | Enable debug logging. |
| `TraceLog` | bool | *Advanced.* Verbose trace logging (beyond `DEBUG`). |
| `ReplayLog` | bool | Write replay logs (for `-replay`). |
| `AHRSLog` | bool | Write AHRS sensor CSV logs (downloadable via `/downloadahrslogs`). |
| `PersistentLogging` | bool | Keep logs across reboots — also makes the filesystem writable (required for dev). |
| `ClearLogOnStart` | bool | *Advanced.* Wipe the debug log on each boot. |
| `DeveloperMode` | bool | Additional SDR diagnostics. Enable via `/develmodetoggle` (one-way) or by tapping the version number; disable via `/setSettings`. |

## Cooling / power / misc

| Field | Type | Description |
|---|---|---|
| `PWMDutyMin` | int | *Advanced.* Minimum fan PWM duty cycle (consumed by the `fancontrol` binary, `fancontrol_main/`). |
| `DarkMode` | bool | Web UI dark theme. |
| `NoSleep` | bool | *Advanced.* Disable [sleep-mode detection](integration/gdl90.md#sleep-mode) for GDL90 clients. Useful for always-on panel-mount EFIS where the display never sleeps. |
| `RegionSelected` | int | `0`=none, `1`=US, `2`=EU. Drives UAT band selection and some OGN behavior. Prefer `POST /setRegion`. |
