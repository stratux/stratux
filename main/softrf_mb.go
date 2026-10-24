/*
	Copyright (c) 2024 Stratux Contributors
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	softrf_mb.go: Integration of Moshe-Braner SoftRF HAT as a subprocess.
	Writes /etc/softrf/settings.txt from globalSettings, launches the SoftRF
	binary, pipes Stratux GPS NMEA into stdin, and reads $PFLAA/$PFLAU from
	stdout into the existing FLARM traffic pipeline.
*/

package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"time"
)

const softRFBinaryPath = "/usr/bin/softrf"
const softRFSettingsPath = "/etc/softrf/settings.txt"

// softRFNMEAChan carries GPS NMEA sentences to the SoftRF subprocess stdin.
var softRFNMEAChan = make(chan string, 100)

// softRFRestartChan signals the subprocess manager to restart with new settings.
var softRFRestartChan = make(chan bool, 1)

// softRFShutdownChan signals the subprocess manager to kill the subprocess and exit.
var softRFShutdownChan = make(chan bool, 1)

// softRFPublishNmea is called from gps.go for every valid NMEA sentence.
// Only GGA and RMC sentences are forwarded — SoftRF's RPi_PickGNSSFix() needs
// both within 600ms to sync its internal clock, and flooding stdin with all
// NMEA types causes the GGA/RMC pair to arrive in different read cycles,
// resulting in a drifting RF_time that breaks FLARM decryption.
func softRFPublishNmea(nmea string) {
	if !globalSettings.SoftRFEnabled {
		return
	}
	// Only forward GGA and RMC — the two sentences needed for GNSS time sync.
	if len(nmea) > 6 {
		talker := nmea[1:3] // e.g. "GP", "GN", "GL"
		_ = talker
		sentenceType := nmea[3:6]
		if sentenceType != "GGA" && sentenceType != "RMC" {
			return
		}
	}
	if !strings.HasSuffix(nmea, "\r\n") {
		nmea += "\r\n"
	}
	select {
	case softRFNMEAChan <- nmea:
	default: // drop if channel is full; GPS updates are frequent enough
	}
}

// softRFSignalRestart asks the running subprocess to restart with updated settings.
// Called from managementinterface.go when SoftRF settings change.
func softRFSignalRestart() {
	select {
	case softRFRestartChan <- true:
	default:
	}
}

// softRFShutdown kills the SoftRF subprocess during graceful shutdown.
func softRFShutdown() {
	select {
	case softRFShutdownChan <- true:
	default:
	}
}

func writeSoftRFSettings() error {
	if err := os.MkdirAll("/etc/softrf", 0755); err != nil {
		return err
	}

	// Resolve int fields, applying defaults for unread (-1) values.
	protocol := 7 // FLARM Latest
	if globalSettings.SoftRFProtocol > 0 {
		protocol = globalSettings.SoftRFProtocol
	}
	altProtocol := 8 // ADS-L
	if globalSettings.SoftRFAltProtocol >= 0 {
		altProtocol = globalSettings.SoftRFAltProtocol
	}
	band := 2 // US 915 MHz
	if globalSettings.SoftRFBand > 0 {
		band = globalSettings.SoftRFBand
	}
	txPower := 2 // full
	if globalSettings.SoftRFTxPower >= 0 {
		txPower = globalSettings.SoftRFTxPower
	}
	alarm := 3 // FLARM algorithm
	if globalSettings.SoftRFAlarm >= 0 {
		alarm = globalSettings.SoftRFAlarm
	}
	relay := 1 // relay landed traffic
	if globalSettings.SoftRFRelay >= 0 {
		relay = globalSettings.SoftRFRelay
	}

	acftType := mapAircraftType(typeMappingOgn2SoftRF, true, globalSettings.OGNAcftType)
	if acftType < 0 {
		acftType = 1 // glider default
	}
	idMethod := globalSettings.OGNAddrType
	aircraftID := globalSettings.OGNAddr
	if len(aircraftID) == 0 {
		aircraftID = "000000"
	}

	stealth := 0
	if globalSettings.SoftRFStealth {
		stealth = 1
	}
	noTrack := 0
	if globalSettings.SoftRFNoTrack {
		noTrack = 1
	}

	// flr_adsl enables simultaneous FLARM Latest + ADS-L reception using a combined
	// 2-byte syncword {0x56, 0x66}. Without flr_adsl, the SX1262 mixed protocol
	// time-slotting is broken and reception drops to near-zero. The Pi's SoftRF binary
	// requires flr_adsl=1 for reliable dual-protocol operation. TX still uses standard
	// per-protocol syncwords, so non-flr_adsl devices can receive the Pi's transmissions.
	flrAdsl := 0
	if (protocol == 7 || altProtocol == 7) && (protocol == 8 || altProtocol == 8) {
		flrAdsl = 1
	}

	content := fmt.Sprintf(
		"SoftRF,1\n"+
			"mode,0\n"+
			"protocol,%d\n"+
			"altprotocol,%d\n"+
			"band,%d\n"+
			"acft_type,%d\n"+
			"id_method,%d\n"+
			"aircraft_id,%s\n"+
			"alarm,%d\n"+
			"relay,%d\n"+
			"tx_power,%d\n"+
			"stealth,%d\n"+
			"no_track,%d\n"+
			"flr_adsl,%d\n"+  // dual FLARM Latest + ADS-L simultaneous reception
			"nmea_out,1\n"+   // output to stdout
			"nmea_g,00\n"+    // no GPS sentences (Stratux has its own GPS)
			"nmea_s,00\n"+    // no sensor sentences
			"nmea_t,01\n"+    // traffic sentences on ($PFLAA/$PFLAU)
			"nmea_e,00\n"+    // no tunnel
			"nmea_d,00\n"+    // no debug sentences
			"gdl90,0\n"+      // Stratux generates GDL90
			"logflight,0\n",  // no flight logging
		protocol, altProtocol, band,
		acftType, idMethod, aircraftID,
		alarm, relay, txPower,
		stealth, noTrack, flrAdsl,
	)

	return os.WriteFile(softRFSettingsPath, []byte(content), 0644)
}

func softRFListen() {
	for {
		if !globalSettings.SoftRFEnabled {
			time.Sleep(1 * time.Second)
			continue
		}

		if _, err := os.Stat(softRFBinaryPath); os.IsNotExist(err) {
			log.Printf("SoftRF HAT: binary not found at %s, waiting...", softRFBinaryPath)
			time.Sleep(30 * time.Second)
			continue
		}

		if err := writeSoftRFSettings(); err != nil {
			log.Printf("SoftRF HAT: failed to write settings: %v", err)
			time.Sleep(5 * time.Second)
			continue
		}

		log.Printf("SoftRF HAT: starting subprocess")
		cmd := exec.Command(softRFBinaryPath)

		stdin, err := cmd.StdinPipe()
		if err != nil {
			log.Printf("SoftRF HAT: stdin pipe error: %v", err)
			time.Sleep(5 * time.Second)
			continue
		}
		stdout, err := cmd.StdoutPipe()
		if err != nil {
			log.Printf("SoftRF HAT: stdout pipe error: %v", err)
			time.Sleep(5 * time.Second)
			continue
		}
		stderr, err := cmd.StderrPipe()
		if err != nil {
			log.Printf("SoftRF HAT: stderr pipe error: %v", err)
			time.Sleep(5 * time.Second)
			continue
		}

		if err := cmd.Start(); err != nil {
			log.Printf("SoftRF HAT: failed to start: %v", err)
			time.Sleep(5 * time.Second)
			continue
		}
		log.Printf("SoftRF HAT: subprocess started (PID %d)", cmd.Process.Pid)

		exitChan := make(chan error, 1)
		go func() { exitChan <- cmd.Wait() }()

		// Writer: forward GPS NMEA from channel to SoftRF stdin.
		go func() {
			for nmea := range softRFNMEAChan {
				if _, err := fmt.Fprint(stdin, nmea); err != nil {
					return
				}
			}
		}()

		// Reader: parse $PFLAA/$PFLAU from SoftRF stdout.
		stdoutScanDone := make(chan struct{}, 1)
		go func() {
			defer func() { stdoutScanDone <- struct{}{} }()
			scanner := bufio.NewScanner(stdout)
			for scanner.Scan() {
				line := scanner.Text()
				if !strings.HasPrefix(line, "$PFL") && !strings.HasPrefix(line, "$PSRFH") {
					if globalSettings.DEBUG && len(line) > 0 {
						log.Printf("SoftRF HAT stdout: %s", line)
					}
					continue
				}
				// Strip checksum suffix before splitting fields.
				bare := line
				if idx := strings.LastIndex(line, "*"); idx >= 0 {
					bare = line[:idx]
				}
				fields := strings.Split(bare, ",")
				if len(fields) < 2 {
					continue
				}
				switch fields[0] {
				case "$PFLAA":
					parseFlarmPFLAA(fields)
					// Count by protocol: the ID field (index 6) has format ADDR!PREFIX_ADDR
					// where PREFIX is "FLR" (FLARM Latest), "FLO" (FLARM Legacy), "ADL" (ADS-L), etc.
					if len(fields) > 6 {
						id := fields[6]
						if bang := strings.Index(id, "!"); bang >= 0 {
							rest := id[bang+1:]
							prefix := rest
							if under := strings.Index(rest, "_"); under >= 0 {
								prefix = rest[:under]
							}
							switch prefix {
							case "FLR", "LND": // FLARM Latest (LND = landed-out Latest)
								globalStatus.SoftRF_rx_FLARM_latest++
							case "FLO": // FLARM Legacy
								globalStatus.SoftRF_rx_FLARM_legacy++
							case "ADL": // ADS-L
								globalStatus.SoftRF_rx_ADSL++
							default:
								globalStatus.SoftRF_rx_other++
							}
						}
					}
					var m msg
					m.MessageClass = MSGCLASS_OGN
					m.TimeReceived = stratuxClock.Time
					msgLogAppend(m)
				case "$PFLAU":
					parseFlarmPFLAU(fields)
				case "$PSRFH":
					// $PSRFH,addr,proto,altproto,millis,voltage,heap,rx_pkts,tx_pkts,nacft,maxrssi
					if len(fields) >= 9 {
						if rx, err := strconv.ParseUint(fields[7], 10, 32); err == nil {
							globalStatus.SoftRF_rx_packets = uint32(rx)
						}
						if tx, err := strconv.ParseUint(fields[8], 10, 32); err == nil {
							globalStatus.SoftRF_tx_packets = uint32(tx)
						}
					}
				}
			}
		}()

		stderrScanDone := make(chan struct{}, 1)
		go func() {
			defer func() { stderrScanDone <- struct{}{} }()
			scanner := bufio.NewScanner(stderr)
			for scanner.Scan() {
				line := strings.TrimSpace(scanner.Text())
				if len(line) > 0 {
					log.Printf("SoftRF HAT stderr: %s", line)
				}
			}
		}()

		// Wait for subprocess exit or a settings-change restart signal.
		select {
		case err := <-exitChan:
			log.Printf("SoftRF HAT: subprocess exited: %v", err)
		case <-softRFRestartChan:
			log.Printf("SoftRF HAT: restarting subprocess for settings change")
			cmd.Process.Kill()
			<-exitChan
		case <-softRFShutdownChan:
			log.Printf("SoftRF HAT: shutting down subprocess")
			cmd.Process.Kill()
			<-exitChan
			stdin.Close()
			<-stdoutScanDone
			<-stderrScanDone
			return
		}

		stdin.Close()
		<-stdoutScanDone
		<-stderrScanDone
		time.Sleep(3 * time.Second)
	}
}
