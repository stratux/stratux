# Modernization Audit

This document records opportunities to modernize the stratux codebase: outdated
dependencies, deprecated language/stdlib usage, end-of-life frameworks, and aging
build/CI/packaging tooling. It is a companion to the [dead-code audit](dead-code.md).

The audit was produced by reference analysis (repo-wide grep, reading the manifests
and build files) cross-checked against upstream release pages and published security
advisories. Nothing here has been changed yet — it is a prioritized backlog. Version
numbers and CVEs reflect what was current at the audit date; re-check before acting.
Line-number citations are snapshot anchors and will drift.

Last updated: 2026-06-08.

## Priority legend

| Tier | Meaning |
| --- | --- |
| 🔴 Security | Known CVE or EOL software with unpatched advisories — address first. |
| 🟢 Quick win | Low risk, mechanical or self-contained; high ROI. |
| 🟡 Needs care | API-breaking, cgo, or hardware-dependent — test before/after. |
| 🔵 Strategic | Large effort; plan as its own project. |

---

## 🔴 Security-critical

| Item | Where | Issue | Tier |
| --- | --- | --- | --- |
| `golang.org/x/net` v0.1.0 → latest | `go.mod` | HTTP/2 Rapid Reset DoS (CVE-2023-39325 / CVE-2023-44487, fixed v0.17.0) plus later CONTINUATION-flood fixes. ~54 minor versions behind. Safe bump. | 🔴 |
| `golang.org/x/text` v0.4.0 → latest | `go.mod` (indirect) | Language-tag parsing DoS (CVE-2022-32149, fixed v0.3.8). Safe bump. | 🔴 |
| `google.golang.org/protobuf` v1.23.0 → latest | `go.mod` (indirect) | `protojson` infinite-loop DoS (CVE-2024-24786, fixed v1.33.0). Resolved transitively by bumping `prometheus/client_golang`. | 🔴 |
| AngularJS 1.4.6 | `web/maui/js/angular*.js` | EOL since Dec 2021; multiple unpatched CVEs (XSS/template injection, ReDoS, sanitizer bypass incl. CVE-2025-0716). Mitigated in practice by device-local Wi-Fi serving, but unambiguously past EOL. Strategic remediation — see Frontend. | 🔴 / 🔵 |
| Unbounded `strcpy` from RF-derived data | `dump978/uat_decode.c:841` | DLAC-decoded text copied into a fixed 1024-byte stack buffer (`report_buf[1024]` at `:829`) with no length check on the fallback path. Untrusted RF input → potential stack overflow. Bound it (`snprintf` / explicit length check). | 🔴 |

`govulncheck` is not currently run anywhere. Installing it
(`go install golang.org/x/vuln/cmd/govulncheck@latest`) and wiring it into CI would
give call-graph-aware confirmation of which advisories are actually reachable.

---

## Go dependencies & toolchain

`go.mod` declares `go 1.20`. Latest Go is 1.24+/1.25.

### Dependency upgrades

| Module | Current | Latest | Tier | Notes |
| --- | --- | --- | --- | --- |
| `golang.org/x/net` | v0.1.0 | v0.55.x | 🟢🔴 | See security table. |
| `golang.org/x/text` | v0.4.0 | v0.37.x | 🟢🔴 | See security table. |
| `golang.org/x/sys` | v0.11.0 | latest | 🟢 | Safe bump. |
| `golang.org/x/exp` | pseudo (2023-07) | latest commit | 🟢 | No stable tags by design; safe to advance. Only one use (`slices.Contains`) — see Go source. |
| `gorilla/websocket` | v1.4.2 | v1.5.3 | 🟢 | Used by the web WebSocket handlers. Safe minor. |
| `sirupsen/logrus` | v1.9.3 | v1.9.4 | 🟢 | Trivial patch. |
| `dustin/go-humanize` | v1.0.0 | v1.0.1 | 🟢 | Trivial patch. |
| `stianeikeland/go-rpio/v4` | v4.5.1-0.2021… (pseudo) | v4.6.0 | 🟢 | A real tag now exists — replace the pseudo-version with `v4.6.0`. |
| `golang/glog` | pseudo (2016) | v1.2.x | 🟢 | Pulled in only by `kidoman/embd`; harmless bump but won't fully drop until embd is replaced. |
| `prometheus/client_golang` | v1.10.0 | v1.23.x | 🟡 | Drops the two deprecated protobuf modules below, but **requires Go ≥ 1.23** (toolchain pivot). API used (`promauto`/`promhttp`) is stable. |
| `mattn/go-sqlite3` | v1.14.6 | v1.14.45 | 🟡 | cgo; core datastore. API-stable patch line but requires rebuild + test. |
| `tinygo.org/x/bluetooth` | v0.10.0 | v0.15.x | 🟡 | BLE adapter API has breaking changes across these. Test GDL90/BLE output on hardware. |
| `BertoldVdb/go-ais` | v0.1.0 | v0.4.x | 🟡 | 0.1→0.4 on a v0.x lib; AIS decode API may have changed. Test the AIS path. |
| `adrianmo/go-nmea` | v1.3.0 | v1.10.x | 🟡 | 7 minor versions of NMEA-parsing additions; verify sentence parsing. |
| `gonum.org/v1/plot` | v0.9.0 | v0.17.x | 🟡 | Diagnostics/plotting only; large jump, pulls newer gonum/x-image. |
| `golang/protobuf` | v1.4.3 | deprecated | 🟡 | Superseded by `google.golang.org/protobuf`. **Disappears when Prometheus is upgraded.** |
| `matttproud/golang_protobuf_extensions` | v1.0.1 | archived | 🟡 | Only present via old `prometheus/common`. **Dropped automatically by Prometheus ≥ ~v1.16.** |

Deps with no newer upstream (custom/forked/abandoned — leave as-is): `jpoirier/gortlsdr`,
`kellydunn/golang-geo`, `takama/daemon`, `tarm/serial`, `stratux/goflying`, `stratux/serial`,
`uavionix/serial`, `felixge/pidctrl`, `gansidui/geohash`, `kidoman/embd`.

### Three serial libraries (consolidation)

`main/` imports **three** serial packages directly:

- `tarm/serial` — `gnss.go`, `clientconnection.go`, `tracker.go`, `network.go`, `pong.go`, `ping.go` (de-facto primary)
- `stratux/serial` — `lowpower_uat.go`, `pong.go` (stratux's own maintained fork)
- `uavionix/serial` — `ping.go` only (an untagged `v0.0.0-19700101…` epoch snapshot)

`pong.go` mixes tarm+stratux; `ping.go` mixes tarm+uavionix. Consolidating onto one
(ideally `stratux/serial`) would remove two modules. Requires source edits + testing. 🟡

### Toolchain

| Aspect | Finding |
| --- | --- |
| `go.mod` directive | `go 1.20`. Recommend bumping to **`go 1.23`** (matches the Prometheus floor; available in bookworm-backports). |
| Dockerfile Go | Installs `golang-go` from apt; Debian Bookworm ships Go 1.21 — **inconsistent with go.mod** and pre-`GOTOOLCHAIN`. Install a pinned upstream Go or use `FROM golang:1.2x-bookworm`. |
| Pi/Debian floor | bookworm-backports ships golang-1.22/1.23/1.24/1.25 — practical floor for a modern toolchain. |
| CI Go | Neither workflow pins a Go version (no `actions/setup-go`). The toolchain is implicit and unasserted against go.mod. |

---

## Go source modernization

Language/stdlib idiom opportunities in `main/`, `common/`, `uatparse/`, `godump978/`,
`sensors/`, `ogn/` (test files and vendored C excluded). These are independent of the
dependency bumps above.

### Mechanical / safe (one PR; unblocks bumping the `go` directive to 1.21)

| Change | Count | Example | Notes |
| --- | --- | --- | --- |
| `io/ioutil` → `os`/`io` | 6 files, ~15 sites | `common/cputemp.go:23`, `main/network.go:63`, `uatparse/uatparse.go` | Deprecated since Go 1.16. `ReadFile`/`WriteFile`/`ReadAll`/`Discard` are renames; the 5 `ReadDir` sites need minor follow-up (`os.ReadDir` returns `[]os.DirEntry`). |
| `interface{}` → `any` | 25 occurrences / 7 files | `main/messagequeue.go`, `main/datalog.go` | `gofmt -r 'interface{} -> any'` handles it. Go 1.18+. |
| Delete dead `IMin`/`IMax` | 2 funcs | `common/equations.go:343,350` | Zero callers (confirms the dead-code audit). Keep `ArrayMin`/`ArrayMax` — those are used. |
| `golang.org/x/exp/slices` → stdlib `slices` | 1 import, 1 call | `main/trace.go:18,149` (`slices.Contains`) | The only x/exp usage in the tree; switching drops the dependency. Go 1.21. |
| Remove redundant legacy build tag | 1 line | `test/uat_read.go:2` | File already has `//go:build`; the old `// +build ignore` line is redundant. |

### Needs judgment

| Change | Where | Notes |
| --- | --- | --- |
| Loop-variable capture (pre-1.22) | `main/gnss.go:1777` | A `go func(){…}()` inside a `range` loop closes over the range variable. A `break` follows so only one goroutine spawns in practice, but it's a latent footgun under 1.20 semantics. Only such case in the tree. |
| Error wrapping | `main/sensors.go:493`, `main/datalog.go:409` | The 2 `fmt.Errorf` calls don't use `%w`; `errors.Is`/`errors.As` are used nowhere. Low value since these errors aren't inspected downstream. Optional. |

### Checked and clean (no action)

`rand.Seed` (none — one benign `rand.Int()`), `strings.Title` (none), `time.Now().Sub`
(none — all `.Sub` calls are on stored timestamps), `os.Setenv` (none), `panic`/`recover`
(only intentional deferred guards + standalone test tools). No other `// +build` lines.

---

## Frontend / web UI

All assets are hand-vendored committed `.min.js` (plus full copies); there is **no build
tooling** (no `package.json`, `bower.json`, webpack/gulp). `web/Makefile` only `cp`s files
and stamps the appcache. `web/index.html` loads ~15 separate `<script>` tags with a literal
`<!-- TODO: combine and minify -->`.

### Library inventory

| Library | Vendored | Latest | Status | Tier |
| --- | --- | --- | --- | --- |
| AngularJS | 1.4.6 (2015) | 1.8.3 (final 1.x) | **EOL Dec 2021**, unpatched CVEs | 🔴🔵 |
| angular-ui-router | 0.2.15 | 1.0.x (`@uirouter/angularjs`) | Deprecated; dies with AngularJS | 🔵 |
| mobile-angular-ui (bundles Bootstrap 3) | ~1.x | 1.3.3 (2016) | Unmaintained ~10 yrs; Bootstrap 3 also EOL | 🔵 |
| OpenLayers (`ol.js`) | ~6.x (2021) | 10.x | Behind, self-contained, no critical CVEs | 🟢 |
| ol-mapbox-style (`olms.js`) | OL6-era | tracks OL 10 | Pairs with OL | 🟢 |
| ol-layerswitcher | OL6-era | 4.x | Pairs with OL | 🟢 |
| svg.js | 2.7.1 | 3.2.5 | One major behind; renders AHRS gauges | 🟢 |
| NoSleep.js | 0.5.0 | 0.12.0 | Behind; replaceable by native Screen Wake Lock API | 🟢 |
| add-to-homescreen (cubiq) | archived build | — | Repo archived Feb 2019; obsolete | 🟢 |
| jQuery | not present | — | Only jqLite refs inside angular.js | — |

CSS: Bootstrap 3 (via mobile-angular-ui). Icons: Font Awesome v4-era.

### AngularJS embedding

Deeply embedded — it is the core of the UI: **11 controllers** (one per screen/plate),
**10 custom directives** (input validators in `settings.js`), ui-router routing in `main.js`,
~300+ `ng-*` directive uses across the HTML plates, ~4,500 LOC of `$scope`/`$http`/`$interval`-coupled
app JS. The two interactive non-Angular pieces — svg.js (AHRS gauges, `ahrs.js`) and
OpenLayers (map/radar) — are framework-independent and would survive a migration unchanged.

Options: (A) bridge to a patched AngularJS 1.8.3 build — small/medium; (B) migrate to a modern
framework (Vue/Svelte/React + Vite) — large→very-large; (C) vanilla JS / Web Components — large.

### PWA / mobile cruft (removable)

Legacy AppCache (`<html manifest="stratux.appcache">` — removed from modern browsers, no
service worker, no `manifest.json`), archived add-to-homescreen, and many pre-iPhone-6
`apple-touch-startup-image` splash tags.

### Frontend quick wins

1. Bump OpenLayers + olms + ol-layerswitcher to current majors (only `map.js`/`radar.js` consume them).
2. Bump svg.js 2.7.1 → 3.2.5 (only `ahrs.js`; reconcile minor v3 API changes).
3. Replace NoSleep.js with the native Screen Wake Lock API (single call site in `gnss.js`).
4. Remove dead PWA cruft; if PWA install is wanted, add a real `manifest.json` + minimal service worker.

---

## Build / CI/CD / Docker / packaging

### GitHub Actions

Both workflows run on `ubuntu-24.04-arm` (native arm64). Action pins are clear of the
deprecation cliff (checkout@v4, upload-artifact@v4, softprops/action-gh-release@v2) — each is
~1 major behind latest but none deprecated.

CI/CD gaps (highest-ROI work):

- **No `.github/dependabot.yml` or renovate config** — add Dependabot for `gomod` + `github-actions` + `docker` (+ submodules). Highest leverage; automates most of this document.
- **No `actions/setup-go`** — Go is whatever the runner ships (implicit, unpinned, uncached). Adding `setup-go@v6` with a pinned version + default caching fixes the toolchain-mismatch *and* adds build caching.
- **No security scanning** — add `govulncheck` (and consider CodeQL).
- **No linting** — no `golangci-lint`/`go vet`; `.golangci.yml` doesn't exist and `.vscode/settings.json` sets `go.lintFlags: ["--disable-all"]`.
- **No tests in CI** — CI builds the `.deb` (`make ddpkg`) but never runs `make test`.
- **No amd64 matrix** — the amd64 build path in the Makefile is never exercised in CI.

### Dockerfile (dev/build container)

Ordered by impact:

1. Pinned Go from apt (`golang-go`) → Bookworm Go 1.21; can't move to a newer toolchain. Install pinned upstream Go or `FROM golang:1.2x-bookworm`.
2. Hardcoded `arm64`-only librtlsdr `.deb`s (`docker-compose.yaml` also pins `platform: linux/arm64/v8`) → image is arm64-only. Parameterize via `TARGETARCH` or document the constraint.
3. 10 separate `RUN apt-get install` layers → consolidate into one `RUN apt-get update && apt-get install -y --no-install-recommends <pkgs> && rm -rf /var/lib/apt/lists/*`.
4. Missing `--no-install-recommends` and cache cleanup.
5. `FROM debian:bookworm` floating tag → pin a digest or dated tag for reproducibility.
6. No checksum verification on `wget`'d `.deb`s (also applies to the image_build scripts) — supply-chain risk.
7. `NOPASSWD: ALL` sudoers for the build user — broad grant; acceptable for a throwaway container but noted.

`docker-compose.yaml` / `docker_run.sh` are otherwise modern (host UID/GID, GOCACHE set).

### Makefile / image build / packaging

- **Makefile:** several targets not declared `.PHONY` (`all`, `clean`, `install`, …); `xrtlais` does a non-idempotent `sed -i` into the rtl-ais submodule's Makefile on every build (dirties the submodule tree, mitigated by `ignore = dirty`). No deprecated `go get` install antipattern present.
- **Stale VSCode configs:** `.vscode/tasks.json` and `launch.json` reference the non-existent `gen_gdl90` target/binary — renamed to `stratuxrun`.
- **Pi image build:** pi-gen submodule pinned to a Jan 2025 Bookworm snapshot (Trixie is the newer line). Heavy `wget` of pinned arm64-only `.deb`s with no checksums. `pip install --break-system-packages esptool` (a venv would be cleaner). `make -j8` hardcoded (use `$(nproc)`).
- **Debian packaging:** hand-rolled `dpkg-deb` (only a `control.dpkg` template; no `debian/rules`, `changelog`, or debhelper-compat). Functional but non-standard; migrating to debhelper would add changelog/versioning/lintian for free — large, low-urgency. systemd units are basically modern; minor: `ExecStopPost=/usr/bin/killall` is blunt, `StandardOutput=null` discards logs, no hardening directives.

---

## Native C / submodules / scripts

### Submodules

| Name | Path | Upstream | Pin (date) | Health |
| --- | --- | --- | --- | --- |
| dump1090 | `dump1090/` | flightaware/dump1090 | v9.0 (2023-11) | Current — latest tagged release. |
| rtl-ais | `rtl-ais/` | mik3y/rtl-ais (fork) | 2021-01 | At the fork's HEAD, but the fork is dormant. Consider migrating the submodule URL to the maintained `dgiardini/rtl-ais` origin (confirm the `-k` TCP-streaming feature exists there first). 🟡 |
| ogn-tracker | `ogn/ogn-tracker/` | pjalocha/ogn-tracker | 2026-02 | Fresh. |
| pi-gen | `image_build/pi-gen/` | RPi-Distro/pi-gen | 2025-01 (Bookworm) | Recent; bump opportunistically toward Trixie. |

`dump978/` is **vendored in-tree** (not a submodule); upstream `mutability/dump978` is itself
deprecated (superseded by `flightaware/dump978-fa`), but a wholesale swap is out of scope —
keep the vendored copy and apply the targeted fixes below.

### C code (dump978)

- **🔴 `uat_decode.c:841`** unbounded `strcpy` from RF-derived DLAC text into a 1024-byte stack buffer (see security table). Highest-value safety fix.
- `uat2json.c:167-169` two `strcpy` into `char[9]` fields — currently size-matched and bounded, but fragile; prefer `snprintf`/bounded `memcpy`. (`:169` copies the callsign field into `squawk` intentionally — worth a comment.)
- **Makefile hardening (cheap):** `CFLAGS = -O2 -Wall -Werror -Ifec` has no `-std=` (pin `-std=gnu11`), no `-Wextra`, no `-D_FORTIFY_SOURCE=2 -fstack-protector-strong`. The `lib:` target duplicates flags rather than reusing `CFLAGS`.
- No `gets`/`sprintf`/`system`/`popen` found. `fec/` (KA9Q Reed-Solomon) is clean — leave it.

### Python (Py3 compatibility)

| File | Verdict | Issues |
| --- | --- | --- |
| `dump978/plot_nexrad.py` | **Py2 only — broken on Py3** | `print` statements, `xrange`, Py2 cairo API. Auxiliary NEXRAD tool, not on the runtime path. Port or remove. |
| `scripts/download_osm_tiles.py` | **Py3-clean** | Runs on 3.12+. Minor: unused `subdomain` var. |
| `test/screen/screen.py` | **Py2 only — broken on Py3** | `import urllib2`, `print x`, `from daemon import runner`. OLED status-display tool. Port or remove. |

### Shell scripts (sampled)

All in-tree scripts are `#!/bin/bash` (so bashisms are legitimate). Common patterns:

- **Missing `set -euo pipefail`** (most prevalent). The firmware flashers (`ogn`/`softrf` `install-*-firmware-pi.sh`) have no error handling — they `systemctl start stratux` even if `esptool.py` fails mid-flash. Add `set -e` + an esptool exit-code check there first.
- `cli.sh` has **no shebang** (first line is a comment).
- Deprecated backticks instead of `$(…)` in `debian/stratux-pre-start.sh`, `stratux-wifi.sh`, `sdr-tool.sh`, `image_build/build.sh`.
- Unquoted variables — `debian/stratux-wifi.sh` is the worst offender (`kill $pid`, `ps -p $pid`, unquoted `$interface`/`$1`).
- `stratux-pre-start.sh` parses `ls -1t | head -1` to pick the newest update file — fragile-on-odd-filenames, but the input is controlled.

---

## Suggested sequencing

1. **Security:** the four dep bumps (x/net, x/text, protobuf-via-Prometheus) + the `dump978/uat_decode.c:841` `strcpy` fix.
2. **Mechanical Go PR:** ioutil→os/io, `interface{}`→`any`, delete `IMin`/`IMax`, x/exp→stdlib `slices`, build-tag cleanup — then bump the `go` directive.
3. **CI automation:** Dependabot + `setup-go` (pinned + cached) + `govulncheck` + run `make test`.
4. **Self-contained frontend bumps:** OpenLayers, svg.js, NoSleep→Wake Lock, remove PWA cruft.
5. **Careful bumps:** Prometheus (→ Go 1.23), go-sqlite3, bluetooth, go-ais, go-nmea, gonum/plot — each tested on hardware.
6. **Strategic:** plan the AngularJS migration as its own project.
