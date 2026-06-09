# Deprecation & Removal Audit

This document tracks **whole features and hardware integrations that are candidates
for outright removal** — as opposed to the [dead-code audit](dead-code.md), which
lists individual unreferenced symbols, and the [modernization audit](modernization.md),
which lists things to upgrade rather than delete.

Each entry records the rationale, a complete removal inventory (every file and symbol
that participates in the feature), the things that must **not** be touched, and a
suggested sequence. Nothing here has been removed yet — these are proposals.
Line-number citations are snapshot anchors and will drift.

Last updated: 2026-06-09.

---

## uAvionix Ping ADS-B receiver

**Proposal:** remove support for the uAvionix **Ping** family of ADS-B receivers
(`ping`, `ping1090`, `pingUSB`, et al.).

### Rationale

- The Ping hardware is **out of production**; fewer than ~100 units are believed to
  exist in the field. The integration serves a vanishingly small install base.
- `main/ping.go` carries the **same six latent concurrency/crash defects** recently
  fixed in `pong.go` (nil `*sync.WaitGroup` deref, nil/unreachable shutdown channel,
  blocking-scanner shutdown path, duplicate `stderr` loop that wrongly trips
  `shutdownES`, double serial-port close, and an `s[0]` empty-line panic). Given the
  install base, **deleting Ping is higher-ROI than porting those fixes into it.**
- Removal also drops a dependency (`github.com/uavionix/serial`, whose only consumer
  is `ping.go`) and simplifies the shared 1090ES ingest gating.

### Effort / risk

Medium effort, low-to-medium risk. The core is a clean delete (one self-contained
driver file), but it is wired into the GDL90 status/capability path, the 1090ES ingest
gating (shared with ES **and** Pong — edit carefully), the settings/status JSON, and
the AngularJS UI. The riskiest edits are the boolean conditions in `traffic.go` that
also gate ES and Pong.

### Removal inventory

#### Core driver — delete outright

| Location | What |
| --- | --- |
| `main/ping.go` (519 LOC) | The entire Ping driver: `initPingSerial`, `pingNetworkRepeater`, `pingNetworkConnection`, `pingSerialReader`, `pingUSBSerialReader`, `pingShutdown`, `pingKill`, `pingWatcher`, `pingInit`; the **MAVLink** layer (`MavlinkTrafficMessageFormat`, `mavLinkFormat`, `mavLinkParse`); and the ping-only globals (`pingSerialConfig`, `pingSerialPort`, `pingWG`, `closeCh`, `pingDeviceModel`, `pingDeviceSuccessfullyWorking`, `dump1090Connection`, `connectionError`, `shutdownPing`). All confirmed referenced nowhere outside `ping.go`. |

> Note: `ping.go` declares a **bare package global** `var closeCh chan int` (`:37`).
> This is distinct from the `closeCh` *struct fields* on the SDR device types in
> `sdr.go` (`e.closeCh`, `u.closeCh`, …) — deleting `ping.go`'s global is safe.
> MAVLink traffic parsing is **Ping-only**; it goes away entirely with this removal.

#### Global state — remove struct fields

| Location | Field |
| --- | --- |
| `main/gen_gdl90.go:1194` | `Ping_Enabled bool` (settings struct) |
| `main/gen_gdl90.go:1281` | `Ping_connected bool` (status struct) |

#### Lifecycle / GDL90 wiring — edit `gen_gdl90.go`

| Location | What | Action |
| --- | --- | --- |
| `main/gen_gdl90.go:1786` | `pingInit()` | delete the call |
| `main/gen_gdl90.go:1632` | `pingKill()` | delete the call |
| `main/gen_gdl90.go:588` | the `if globalSettings.Ping_Enabled` GDL90 capability-bitmask block that ORs the ES+UAT bits into `msg[13]` ("Ping provides ES and UAT") | delete the block (Pong sets the same bits in the block immediately below) |

#### Shared 1090ES ingest gating — careful edits (keep ES + Pong)

These conditions also gate ES and Pong; **drop only the `Ping_Enabled` term**, do not
delete the lines.

| Location | Current | After |
| --- | --- | --- |
| `main/traffic.go:1043` (`esListen` guard) | `!ES_Enabled && !Ping_Enabled && !Pong_Enabled` | `!ES_Enabled && !Pong_Enabled` |
| `main/traffic.go:1055` (`esListen` loop) | `ES_Enabled \|\| Ping_Enabled` | `ES_Enabled` |
| `main/traffic.go:1089` (`parseDump1090Message`) | `!Ping_Enabled && !Pong_Enabled` | `!Pong_Enabled` |

#### Settings handler

| Location | What | Action |
| --- | --- | --- |
| `main/managementinterface.go:425-426` | `case "Ping_Enabled": globalSettings.Ping_Enabled = val.(bool)` | delete the case |

#### Web UI (AngularJS)

| Location | What | Action |
| --- | --- | --- |
| `web/plates/settings.html:418-420` | "Ping ADS-B" `<ui-switch ng-model='Ping_Enabled' ng-disabled='Pong_Enabled'>` row | delete the row |
| `web/plates/settings.html:427` | Pong switch's `ng-disabled='Ping_Enabled'` | drop the `ng-disabled` (no longer mutually exclusive) |
| `web/plates/js/settings.js:265` | `'Ping_Enabled'` in the `toggles` array | remove the entry |
| `web/plates/js/settings.js:301` | `$scope.Ping_Enabled = settings.Ping_Enabled;` | remove |
| `web/plates/status.html:29-32` | "Ping device" Connected/Disconnected indicator row (`visible_ping`, `Ping_connected`) | delete the row |
| `web/plates/js/status.js:47` | `$scope.Ping_connected = status.Ping_connected;` | remove |
| `web/plates/js/status.js:244,246` | `$scope.visible_ping = settings.Ping_Enabled;` and its guard | remove |
| `web/plates/js/developer.js:46` | `$scope.Ping_connected = status.Ping_connected;` | remove |

#### Dependency

| Location | What |
| --- | --- |
| `main/ping.go:30` → `go.mod` / `go.sum` | `github.com/uavionix/serial` — **sole** consumer is `ping.go`. Drop it after the file is gone. Cross-references the modernization audit's *"Three serial libraries"* item: this takes `main/` from 3 serial libs to 2. |

#### udev / packaging

| Location | What | Action |
| --- | --- | --- |
| `debian/99-uavionix.rules:3,7,10` | FTDI rules creating the `ping` (idProduct `74f0`/`74f1`) and `pingusb` (`6015`) symlinks | remove the Ping lines. The file **currently defines only Ping** devices. **Verify** Pong's `/dev/pong` symlink rule lives in a separate rules file before deleting `99-uavionix.rules` wholesale. |

#### Documentation

Ping is mentioned in `docs/settings-reference.md`, `docs/http-api.md`,
`docs/integration/gdl90.md`, `docs/hardware/README.md`,
`docs/hardware/ogn-ais-receivers.md`, and `docs/hardware/sensors.md` — prune the Ping
references when the feature goes. The existing `dead-code.md` / `modernization.md`
notes about `ping.go` (e.g. the serial-library mix) become moot.

### Do **not** touch — false positives

- **Network client keepalive.** `LastPingResponse`, `pingResponse`, and "ping time"
  in `main/network.go`, `main/clientconnection.go`, and `main/tracker.go` are the
  TCP/UDP client liveness mechanism — unrelated to the Ping device. Leave them.
- **Pong.** Pong is **completely different hardware** — *not* a successor to or
  replacement for Ping. `main/pong.go` is an independent driver (its own
  `closeChpong`, `shutdownPong`, `dump1090ConnectionPong` globals) that merely reuses
  the *same* dump1090 repeater plumbing (`--net-stratux-port 30006` →
  `127.0.0.1:30001`). That shared pipeline is the only reason the two are
  mutually-exclusive in the settings UI (`Ping_Enabled` and `Pong_Enabled` each
  `ng-disable` the other — `web/plates/settings.html:420,427`); it is a plumbing
  constraint, not a product relationship. Removing Ping must leave Pong's ingest path
  intact; confirm Pong still works after the `traffic.go` edits.

### Suggested sequencing

1. Delete `main/ping.go`; remove `pingInit()`/`pingKill()` calls and the `:588`
   capability block in `gen_gdl90.go`; remove the two struct fields.
2. Edit the three `traffic.go` conditions (drop the `Ping_Enabled` term only) and the
   `managementinterface.go` settings case. Build; confirm ES and Pong still ingest.
3. Strip the Web UI (settings toggle, status row, developer/status bindings); remove
   the Pong switch's now-stale `ng-disabled`.
4. Drop `github.com/uavionix/serial` from `go.mod`/`go.sum` (`go mod tidy`).
5. Remove the Ping udev rules (after confirming the Pong rule is elsewhere) and prune
   the doc references.

---

## SiRFstar IV GPS via Prolific PL2303 (BU-353-S4)

**Proposal:** remove support for the legacy **SiRFstar IV** GPS receiver — the GlobalSat **BU-353-S4**, connected over a Prolific **PL2303** USB-serial bridge (internal type `GPS_TYPE_PROLIFIC`, symlink `/dev/prolific0`).

### Rationale

- The SiRFstar IV is an **EOL chipset**: NMEA-only, GPS/SBAS-only (no GLONASS/Galileo/BeiDou/QZSS), and slow to update — categorically inferior to the u-blox 6/7/8/9 lineup that dominates the install base and gets all the real configuration effort (`CFG-GNSS` multi-constellation, airborne <2g dynamic model, 1–10 Hz nav rate). The original Prolific PL2303 is **also EOL** and heavily cloned (counterfeit chips that the genuine Windows/macOS drivers actively reject), so even the USB bridge is a support liability.
- The SiRF branch is a **niche legacy path**: it is the only GPS the code drives via raw `$PSRF…` NMEA command writes, with a one-off baud-probe list and a one-off config block, all gated by a local `isSirfIV` boolean. None of it is shared with the u-blox or generic-serial paths.
- Removal deletes the only consumer of the `makeNMEACmd()` checksum helper and lets the `/dev/prolific0` autodetect branch and its bespoke `4800, 38400, 9600` baud list go away, trimming the autodetect ladder in `initGPSSerial()`.

### Effort / risk

Low effort, **low-to-medium risk**, with one genuinely careful edit. The Go side is a clean delete (one autodetect branch + one config block + a now-orphaned helper). The risk is concentrated in the **udev rule**: the `067b:2303` → `/dev/prolific%n` symlink is, per the in-file comment, shared between the BU-353-S4 GPS **and** the TU-S9 USB-serial adapter (the chip is "indistinguishable using idVendor and idProduct"). That rule must be edited deliberately, not blind-deleted — see false positives. The other care item is the `status.js` GPS-type enumeration, which is manually kept in sync with `gen_gdl90.go` by index, so the `case 2` entry should be retired rather than renumbered.

### Removal inventory

#### Core driver — delete the SiRF branch and its config block in `gps.go`

| Location | What | Action |
| --- | --- | --- |
| `main/gps.go:296-303` | the `else if _, err := os.Stat("/dev/prolific0")` autodetect branch (sets `isSirfIV = true`, the SiRF-only `baudrates = []int{4800, 38400, 9600}`, `device = "/dev/prolific0"`, `globalStatus.GPS_detected_type = GPS_TYPE_PROLIFIC`) | delete the whole `else if` arm |
| `main/gps.go:340-361` | the `if isSirfIV { … }` config block: the seven `p.Write(makeNMEACmd("PSRF…"))` calls (5 Hz, GGA, GSA, RMC, VTG, GSV, and the `PSRF100,1,38400…` baud switch), `baudChanged = true`, and the SiRF debug log | delete the `if isSirfIV` arm; the chain becomes `if (UBX…) { … } else if SOFTRF_DONGLE { … }` |
| `main/gps.go:234` | `isSirfIV := bool(false)` local | delete the declaration |
| `main/gps.go:206-212` | `makeNMEACmd(cmd string) []byte` | delete — a generic `$…*cs\r\n` NMEA-checksum builder whose **only callers are the seven SiRF writes above**. Confirmed zero other callers repo-wide; it becomes dead with the branch. (u-blox uses `makeUBXCFG`; OGN/SoftRF in `tracker.go` use `appendNmeaChecksum`, a *separate* helper — do not touch that.) |

> Note: `GPS_detected_type == GPS_TYPE_PROLIFIC` is **set but never read** as a branch
> condition anywhere — the only behavioral gate is the local `isSirfIV` bool. So no
> `GPS_detected_type ==` comparison needs editing (the u-blox/SoftRF arms at
> `:362-459` are untouched). The value only surfaces as a status string (below).

#### Type constant

| Location | What | Action |
| --- | --- | --- |
| `main/gen_gdl90.go:101` | `GPS_TYPE_PROLIFIC = 2` (lower-nibble GPS-type enum) | delete the constant. Index **2** then becomes unused; **do not renumber** the other `GPS_TYPE_*` values — they are wire/JSON values mirrored in `status.js` (and the comment at `:94-96` warns they exist "for historical reasons" and must stay backward-compatible). Leaving a gap at `2` is correct. |

> The commented-out `GPS_TYPE_SIRF = 0x03` at `gen_gdl90.go:88` is inside a dead
> historical comment block (a different, never-compiled enum) — leaving it is fine,
> though it may be swept by the dead-code audit separately. It is **not** `GPS_TYPE_PROLIFIC`.

#### Web UI (AngularJS)

| Location | What | Action |
| --- | --- | --- |
| `web/plates/js/status.js:99-101` | `case 2: tempGpsHardwareString = "Prolific USB-serial bridge"; break;` in the `gpsHardwareCode` switch (kept "in sync with the enumeration in gen_gdl90.go") | delete the `case 2` block. Do **not** renumber the other cases — they map to the wire values above. |

#### udev / packaging — **careful edit, not a blind delete**

| Location | What | Action |
| --- | --- | --- |
| `debian/10-stratux.rules:26` | `SUBSYSTEMS=="usb", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="prolific%n"` | this `prolific%n` symlink is the BU-353-S4's only door into the daemon (`gps.go` stats `/dev/prolific0`). With the GPS branch gone, nothing in the daemon consumes `/dev/prolific*` (the serialout path in `network.go:141-142` enumerates `/dev/serialout*` and `/dev/serialout_nmea*`, **not** `/dev/prolific*`). So the rule can be removed **for GPS purposes** — but see the false-positive note: the same rule/comment is the only thing documenting the TU-S9 case. Decide whether TU-S9 stays supported before removing; if it stays, leave the rule (and its `:23-24` comment) and only drop the BU-353-S4 mention. |
| `debian/10-stratux.rules:23-24,28-29` | comment "the BU-353-S4 and the TU-S9 (serialout) use the pl2303" and the two commented-out `bu353s4` / `tu-s9` SYMLINK lines | prune the BU-353-S4 wording / the commented `bu353s4` line as part of the same edit; keep TU-S9 wording iff TU-S9 support is retained |

#### Documentation

| Location | What | Action |
| --- | --- | --- |
| `docs/hardware/gps.md:4` | intro sentence "…but a SiRF receiver, a Raspberry Pi UART GPS…" | drop "a SiRF receiver," |
| `docs/hardware/gps.md:17` | the `SiRFstar IV — BU-353-S4 (Prolific PL2303) … GPS_TYPE_PROLIFIC` row in the autodetect table | delete the row |
| `docs/hardware/gps.md:42` | "SiRF tries `4800, 38400, 9600`." bullet under baud detection | delete the bullet |
| `docs/hardware/README.md:39` | `Prolific PL2303 (BU-353-S4 GPS, TU-S9 serial) … 067b:2303 … /dev/prolific*` udev table row | drop the BU-353-S4 GPS half; keep/adjust the row only if TU-S9 support is retained, otherwise delete |

> No references exist in `docs/settings-reference.md` (the `GpsManualChip` field at
> `:70` accepts only `ublox6/7/8/9/10/ublox` — there is **no** SiRF manual-config value,
> so no manual path reaches the SiRF code) or in `docs/http-api.md`. `docs/integration/gdl90.md:130`
> only shows an example `GPS_detected_type` value and enumerates nothing — leave it.

### Do **not** touch — false positives

- **OGN-Tracker / SoftRF `$PSRF…` sentences.** `main/tracker.go:338-445` writes `$PSRFH`, `$PSRFS`, `$PSRFC` and reads `PSRFH`/`PSRFS`, and `docs/hardware/ogn-ais-receivers.md:44` lists `$PSRFH`/`$PSRFS` for SoftRF. These are **SoftRF/OGN proprietary** configuration sentences — *not* SiRF GPS (`$PSRF1xx`). They use the separate `appendNmeaChecksum` helper and the `GPS_TYPE_SERIAL`/OGN path. Leave them entirely.
- **The `MTK or SIRF` comments.** `main/gps.go:1263` and `:1413` (`//30 seconds @ … for MTK or SIRF…`) are explanatory comments inside the **shared** GPS-perf-stats trim logic that runs for every chip. They reference the message cadence, not a SiRF code path. Leave the code; the comments are harmless (optionally drop the word "SIRF").
- **The TU-S9 serial adapter.** The `067b:2303` udev rule is shared between the BU-353-S4 GPS **and** the TU-S9 USB-serial adapter — same Prolific PL2303 chip, "indistinguishable using idVendor and idProduct" (`10-stratux.rules:23-24`). Removing SiRF GPS support does **not** by itself remove TU-S9 support; if TU-S9 is still wanted, the `prolific%n` rule must **stay** and only the BU-353-S4 wording is pruned. This is the one edit that must not be a blind delete.
- **u-blox and generic-serial GPS paths.** Everything in `initGPSSerial()` outside the two SiRF arms — the `/dev/ublox{6,7,8,9}` autodetect branches, `/dev/serial0` (`GPS_TYPE_UBX_GEN`), `/dev/serialin` (`GPS_TYPE_SERIAL`), `/dev/softrf_dongle`, `GPS_TYPE_NETWORK`, the manual-config switch, `detectOpenSerialPort()` itself, and the per-chip `writeUblox8/9ConfigCommands` — is untouched. In particular `detectOpenSerialPort()` is **generic**: the SiRF-specific part is only the *baud list* passed in (`{4800, 38400, 9600}` at `:301`); the function is shared by all paths and stays.

### Suggested sequencing

1. In `gps.go`, delete the `/dev/prolific0` autodetect arm (`:296-303`), the `if isSirfIV` config block (`:340-361`), the `isSirfIV` local (`:234`), and the now-orphaned `makeNMEACmd()` helper (`:206-212`). Build; confirm u-blox, generic-serial, SoftRF, and network GPS still init.
2. Remove the `GPS_TYPE_PROLIFIC = 2` constant in `gen_gdl90.go` (leave the index gap; do not renumber) and the matching `case 2` in `status.js` (do not renumber the other cases).
3. Decide TU-S9's fate, then edit `debian/10-stratux.rules`: if TU-S9 stays, keep the `prolific%n` rule and only prune the BU-353-S4 wording/commented `bu353s4` line; if not, remove the rule and its comment block.
4. Prune the SiRF/BU-353-S4 references in `docs/hardware/gps.md` (intro line, table row, baud bullet) and the BU-353-S4 half of the `docs/hardware/README.md` udev row.

---

## InvenSense / TDK MPU-925x / MPU-6xxx IMU family driver

**Proposal:** remove support for the discontinued InvenSense/TDK **MPU-9250 / MPU-9255 / MPU-9150 / MPU-6500 / MPU-6050 / MPU-6000** IMU family (the `MPU9250` driver and its detection branch), keeping the in-production **ICM-20948** as the sole supported IMU.

### Rationale

- The MPU-9250 and MPU-9150 are **end-of-life / discontinued** at TDK InvenSense (the MPU-6xxx parts likewise long superseded). The integration targets out-of-production silicon.
- Stratux **already supports the modern in-production replacement**, the InvenSense **ICM-20948**, via `sensors/icm20948.go`. The two drivers are near-identical (`ICM20948` is a line-for-line clone of `MPU9250` against the `goflying/icm20948` subpackage), so the supported AHRS path remains fully intact after removal.
- The two chips are distinguished cleanly at probe time: ICM-20948 answers WHO_AM_I `0xEA` at reg `0x00`, while the MPU family answers at reg `0x75` (values `0x71`/`0x73`/`0x70`/`0x68`). The MPU branch can be excised without touching the ICM-20948 branch.
- Removal stops importing the `github.com/stratux/goflying/mpu9250` subpackage. Note this does **not** drop the `stratux/goflying` module from `go.mod` — its `icm20948`, `ahrs`, and `ahrsweb` subpackages are still consumed (see *Do not touch*).

### Effort / risk

Low-to-medium effort, low-to-medium risk. The driver file (`sensors/mpu9250.go`) is a clean self-contained delete with a single caller. The real care is in `main/sensors.go`: the `0x68` WHO_AM_I probe is **shared** with the ICM-20948 (whose WHO_AM_I lives at a *different register*, `0x00`), so the edit is **surgical** — drop the MPU `else if` branch and its constants/caller, but keep the ICM-20948 branch and the entire AHRS pipeline. There is no settings flag, status field, or Web UI element specific to the MPU (the generic `IMU_Sensor_Enabled` toggle covers both chips and stays), so the UI is untouched. Field impact is limited to users running a legacy GY-91/MPU-925x board, who would need to migrate to an ICM-20948 board.

### Removal inventory

#### Core driver — delete outright

| Location | What |
| --- | --- |
| `sensors/mpu9250.go` (99 LOC) | The entire MPU driver: the `MPU9250` struct, `NewMPU9250`, the `Read`/`ReadOne`/`Close` `IMUReader` methods, the `mpu9250GyroRange`/`mpu9250AccelRange`/`mpu9250UpdateFreq` consts, and the sole import of `github.com/stratux/goflying/mpu9250`. Confirmed referenced nowhere outside this file and the single caller in `main/sensors.go`. |

#### IMU detection / probe — careful edits to `main/sensors.go` (keep the ICM-20948 branch)

The MPU and ICM-20948 share the `0x68` probe in `initIMU()`. **Drop only the MPU branch and the MPU-only WHO_AM_I machinery; keep the ICM-20948 branch.**

**WHO_AM_I constants** — remove the MPU-only values (`main/sensors.go:28-33`), keep the ICM values:

| Location | Current | After |
| --- | --- | --- |
| `main/sensors.go:28-33` | `MPUREG_WHO_AM_I = 0x75`, `MPUREG_WHO_AM_I_VAL = 0x71`, `MPUREG_WHO_AM_I_VAL_9255 = 0x73`, `MPUREG_WHO_AM_I_VAL_6500 = 0x70`, `MPUREG_WHO_AM_I_VAL_60X0 = 0x68`, `MPUREG_WHO_AM_I_VAL_UNKNOWN = 0x75`, then `ICMREG_WHO_AM_I = 0x00`, `ICMREG_WHO_AM_I_VAL = 0xEA` | delete all six `MPUREG_*` consts; keep `ICMREG_WHO_AM_I`, `ICMREG_WHO_AM_I_VAL`, and `PRESSURE_WHO_AM_I` |

**Probe body** — `initIMU()` (`main/sensors.go:180-215`):

| Location | Current | After |
| --- | --- | --- |
| `main/sensors.go:181` | `// Check if the chip is the ICM-20948 or MPU-9250.` | `// Check if the chip is the ICM-20948.` |
| `main/sensors.go:187-191` | second read `v2, err := i2cbus.ReadByteFromReg(0x68, MPUREG_WHO_AM_I)` + its error check (used only by the MPU branch) | delete the `v2` read and its error block |
| `main/sensors.go:200-208` | the `else if v2 == MPUREG_WHO_AM_I_VAL \|\| … { log.Printf("MPU detected …"); imu, err := sensors.NewMPU9250(&i2cbus); … }` branch | delete the entire `else if` branch (this drops the `sensors.NewMPU9250` caller) |
| `main/sensors.go:209-212` | `} else { log.Printf("Could not identify MPU. v=%02x, v2=%02x.\n", v, v2); return false }` | keep the fallthrough, but reword to reference only `v` (drop `v2`): `log.Printf("Could not identify IMU. v=%02x.\n", v)` |

Resulting `initIMU()` keeps the single `0x68`/reg-`0x00` read, the `if v == ICMREG_WHO_AM_I_VAL { … sensors.NewICM20948(&i2cbus) … }` branch, and the `else` failure path — nothing else in the function changes.

#### Dependency

| Location | What |
| --- | --- |
| `sensors/mpu9250.go:6` → `go.mod` | `github.com/stratux/goflying/mpu9250` — the import disappears with the file. **The `stratux/goflying` *module* stays in `go.mod`/`go.sum`** (`:18`, `v0.0.0-20250123172850-…`): its `icm20948`, `ahrs`, and `ahrsweb` subpackages are still imported. Run `go mod tidy` after removal; expect **no** module to be dropped (only one fewer subpackage compiled). This is unlike the Ping removal, which dropped a whole module. The modernization audit lists `stratux/goflying` under *"deps with no newer upstream — leave as-is"*; that line is unaffected. |

#### Documentation

| Location | What | Action |
| --- | --- | --- |
| `docs/hardware/sensors.md:29-38` | the IMU WHO_AM_I table rows for MPU-9250 / 9255 / 6500 / 6000/6050/9150 / "Unknown MPU on some GY-91 boards", and the sentence *"The MPU-925x / 615x family all route through the `MPU9250` driver."* | delete the MPU rows (keep the ICM-20948 row); reword the routing sentence to describe only the ICM-20948 path |
| `docs/settings-reference.md:33` | `IMU_Sensor_Enabled … (ICM-20948, MPU-9250 family).` | drop the *"MPU-9250 family"* clause; leave the flag itself |
| `sensors/imu.go:4-6` | doc comment on `IMUReader`: *"such as the InvenSense MPU9150 or MPU9250 … the current github.com/westphae/goflying MPU9250 driver"* | reword the comment to reference the ICM-20948 / `stratux/goflying` driver. **The interface itself stays — comment-only edit.** |

### Do **not** touch — false positives

- **ICM-20948 driver (`sensors/icm20948.go`).** The in-production replacement and the whole point of this proposal — it **stays**, along with its `goflying/icm20948` import and the ICM branch in `initIMU()`.
- **`IMUReader` interface (`sensors/imu.go`) and `PressureReader` (`sensors/pressure.go`).** Both stay; `MPU9250` is merely one implementer being removed. Only the stale doc comment on `IMUReader` (which names MPU9150/MPU9250) is reworded.
- **BMP baro drivers (`sensors/bmp280.go`, `sensors/bmp388/`).** Unrelated pressure-sensor path probed at `0x76`/`0x77` — leave entirely. The `bmp388` dead-code items in `dead-code.md:52-54` are unaffected.
- **The entire AHRS pipeline in `main/sensors.go`.** Calibration (`cal`, `SetCalibrations`, `/calibrateAHRS`, `/cageAHRS`), the quaternion (`SensorQuaternion`), the G-meter (`AHRSGLoad`, `/resetGMeter`), the IMU polling loop (`pollSensors`, `sensorAttitudeSender`, `updateAHRSStatus`), `globalStatus.IMUConnected`, and `globalSettings.IMU_Sensor_Enabled` all **stay** — this is *not* a removal of IMU/AHRS support, only of the MPU chip branch.
- **`IMU_Sensor_Enabled` Web UI.** `web/plates/settings.html`, `web/plates/js/settings.js`, and `web/plates/gps.html`/`js/gps.js` reference the generic IMU toggle and the `status-imu`/ATT indicator. There is **no MPU-specific** UI element — none of these is touched.
- **`goflying` module and `westphae` references.** The `// TODO westphae` note in `main/sensors.go:365` and the `stratux/goflying` module entry in `go.mod` are not removed (the module still serves ICM-20948/AHRS).

### Suggested sequencing

1. Delete `sensors/mpu9250.go`.
2. In `main/sensors.go`: remove the six `MPUREG_*` WHO_AM_I consts; in `initIMU()` delete the `v2` read + error block and the `sensors.NewMPU9250` `else if` branch, and reword the final `else` failure log to drop `v2`. Build; confirm `initIMU()` still compiles and the ICM-20948 branch is intact.
3. Run `go mod tidy`; confirm `stratux/goflying` **remains** in `go.mod`/`go.sum` (only the `mpu9250` subpackage stops compiling).
4. On hardware (or with a known ICM-20948 board), verify `IMU_Sensor_Enabled` still detects the ICM-20948, AHRS attitude (`AHRSPitch`/`AHRSRoll`/`AHRSGyroHeading`), the G-meter, and calibration (`/calibrateAHRS`, `/cageAHRS`) all still work.
5. Prune the MPU rows/sentences from `docs/hardware/sensors.md` and `docs/settings-reference.md`, and reword the `sensors/imu.go` interface comment.

---

## FlightBox / Merlin hardware-build branding

**Proposal:** remove the discontinued prebuilt-unit branding detection
(`/etc/FlightBox`, `/etc/Merlin`) and the `HardwareBuild` status field it feeds.

### Rationale

- **FlightBox** (Open Flight Solutions) and **Merlin** were prebuilt commercial
  Stratux-based units; both are **out of production**. Per
  `docs/hardware/README.md` these are *"branding flags for prebuilt units, not
  receiver integrations"* — there is no receiver/driver behind them, just a label.
- `HardwareBuild` is **write-only**: the *only* code that ever assigns it is the
  two `os.Stat` probes in `gen_gdl90.go` (`= "FlightBox"` / `= "Merlin"`). No Go
  code reads it back, and **no web UI plate references it at all** (grep of
  `web/plates/` for `HardwareBuild` returns nothing — it is neither displayed nor
  used for skinning/conditional UI). It exists solely as a field in the
  `/getStatus` JSON for external consumers, and in the field it is essentially
  always the empty string.
- Because FlightBox/Merlin are the *only* producers, once they go the field can
  **never be non-empty**. Recommendation: **remove the `HardwareBuild` field as
  well** rather than ship a vestigial always-empty string. It is unauthenticated
  status JSON with no internal reader, so dropping it is low-risk; if strict
  wire-compat for third-party `/getStatus` scrapers is a concern, the fallback is
  to keep the field declaration and simply delete the two assignments (it then
  serializes as `""` forever). State the choice explicitly in the changelog.
- The FlightBox probe also carries a **logging side effect**: when `/etc/FlightBox`
  exists it redirects all logs to `/root/` via `logDirf = logDir_FB`
  (`gen_gdl90.go:1716`). Removing FlightBox detection retires that redirect and the
  `logDir_FB` constant; on stock Stratux logging already uses the `else` branch
  (`/var/log/`), so behavior is unchanged for every non-FlightBox unit.

### Effort / risk

Small effort, low risk. This is a clean delete with no driver, no concurrency
surface, and no UI branching to untangle. The only non-trivial point is the
`logDirf`/`logDir_FB` log-path side effect (keep `logDirf` — it's used widely in
`logging.go`/`sensors.go` — and just hard-wire it to `logDir`). The only
externally observable change is the `/getStatus` JSON shape (`HardwareBuild`
removed or permanently `""`).

### Removal inventory

#### Detection code — edit `gen_gdl90.go`

| Location | What | Action |
| --- | --- | --- |
| `main/gen_gdl90.go:1713-1719` | the `/etc/FlightBox` `os.Stat` block that sets `HardwareBuild = "FlightBox"` **and** `logDirf = logDir_FB` | delete the FlightBox branch; keep the log-dir assignment by collapsing it to `logDirf = logDir` |
| `main/gen_gdl90.go:1720-1723` | the `/etc/Merlin` `os.Stat` block that sets `HardwareBuild = "Merlin"` | delete the block |
| `main/gen_gdl90.go:53-54` | `//FlightBox: log to /root.` comment and `logDir_FB = "/root/"` const | delete (sole consumer was the FlightBox branch above) |

#### Status struct field

| Location | Field | Action |
| --- | --- | --- |
| `main/gen_gdl90.go:1260` | `HardwareBuild string` (status struct) | remove — never read in Go, never referenced in the UI. (Fallback: keep the declaration, drop only the two assignments, to preserve the JSON key as `""`.) |

#### Web UI (AngularJS)

| Location | What |
| --- | --- |
| — | **None.** No plate or controller in `web/plates/` references `HardwareBuild`. No status/settings binding, no logo/title/CSS branch, no `ng-if`/`ng-show` gate. Nothing to edit. |

#### Packaging / image

| Location | What |
| --- | --- |
| — | **None.** No `/etc/FlightBox` or `/etc/Merlin` marker file is created anywhere in `debian/`, `image_build/`, `scripts/`, or `common/` (those files were dropped onto units by the OEM image builds, which live outside this repo). Nothing to edit. |

#### Documentation

| Location | What | Action |
| --- | --- | --- |
| `docs/hardware/ogn-ais-receivers.md:97-101` | the *"Hardware-build branding"* section describing `/etc/FlightBox` / `/etc/Merlin` → `HardwareBuild` | delete the section |
| `docs/integration/gdl90.md:113` | `"HardwareBuild": "",` line in the `/getStatus` example JSON | remove the line (or leave it only if the field-retention fallback is chosen) |

### Do **not** touch — false positives

- **`Version` / `Build`.** `globalStatus.Version` (`stratuxVersion`) and
  `globalStatus.Build` (`stratuxBuild`) at `gen_gdl90.go:1709,1711` (struct fields
  `:1258-1259`) are the firmware version and git-commit build hash — **not**
  hardware branding. They are real, populated, externally consumed fields. Leave
  them entirely; only `HardwareBuild` is branding.
- **`logDirf`.** The package-global `logDirf` (`gen_gdl90.go:45`) is the live
  logging directory and is consumed throughout `logging.go` and `sensors.go`. Keep
  it — only the FlightBox-specific `logDir_FB` redirect goes away. After the edit
  `logDirf` is unconditionally `logDir`.
- The `//FlightBox: detect via presence of...` comments are the only in-tree
  mention of these products; there is no separate config struct field, settings
  toggle, or udev rule to chase. This removal does not touch any receiver driver.

### Suggested sequencing

1. In `gen_gdl90.go`, delete the `/etc/Merlin` block and collapse the
   `/etc/FlightBox` block down to `logDirf = logDir` (removing both `HardwareBuild`
   assignments and the `logDir_FB` redirect); delete the `logDir_FB` const and its
   comment.
2. Remove the `HardwareBuild` field from the `status` struct (or, if preserving
   wire-compat, keep the field and stop here for code). Build; confirm logging
   still targets `/var/log/` and `/getStatus` serializes cleanly.
3. Prune the docs: drop the *"Hardware-build branding"* section in
   `docs/hardware/ogn-ais-receivers.md` and the `HardwareBuild` line in the
   `docs/integration/gdl90.md` example JSON.
