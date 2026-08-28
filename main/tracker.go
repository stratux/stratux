package main

import (
	"fmt"
	"log"
	"strconv"
	"strings"
	"time"

	"github.com/tarm/serial"
)

/*
	Copyright (c) 2024 Adrian Batzill
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	---
	tracker.go: Implements handling and configuration of the various tracker devices (OGN Tracker, GXAirCom, SoftRF)

	TODO: SoftRF dongle is not yet considered a tracker, since we use the Moshe-Braner SoftRF fork
*/

var typeMappingOgn2Gx = [][]int {
	{1, 4}, // (Motor)glider
	{8, 5}, // Powered
	{3, 6}, // Helicopter
	{6, 2}, // Hang glider
	{7, 1}, // Paraglider
	{11, 3}, // Balloon
	{13, 7}, // UAV

	{2, 5}, // Tow->Powered
	{4, 1}, // Parachute->Paraglider
	{5, 5}, // Drop plane->Powered
	{9, 5}, // Jet -> Powered
	{10, 0}, // Ufo -> Other
	{12, 3}, // Airship -> Balloon
	{14, 0}, // Ground support
	{15, 0}, // Static object
}

var typeMappingOgn2SoftRF = [][]int {
	{1, 1}, // (Motor)glider
	{2, 2}, // Tow
	{3, 3}, // Helicopter
	{6, 6}, // Hang glider
	{7, 7}, // Paraglider
	{8, 8}, // Powered
	{11, 11}, // Balloon
	{13, 13}, // UAV
	{14, 16}, // Ground support/winch
	{15, 15}, // Static object

	{4, 7}, // Parachute->Paraglider
	{5, 8}, // Drop plane->Powered
	{9, 8}, // Jet -> Powered
	{10, 8}, // Ufo -> Powered
	{12, 11}, // Airship -> Balloon
}

func mapAircraftType(mapping [][]int, forward bool, acType int) int {
	for _, m := range mapping {
		if forward {
			if m[0] == acType {
				return m[1]
			}
		} else {
			if m[1] == acType {
				return m[0]
			}
		}
	}
	return -1
}



// Main interface that all tracker implementations need to implement
type Tracker interface {
	initNewConnection(serialPort *serial.Port)
	onNmea(serialPort *serial.Port, nmea []string) bool
	gpsTimeOffsetPps() time.Duration
	getGpsHardwareType() uint
	isDetected() bool
	isConfigRead() bool
	// How long should we wait after a write operation until we read back data
	writeReadDelay() time.Duration
	// Done on every new connection, return true if something was written
	writeInitialConfig(serialPort *serial.Port) bool

	requestTrackerConfig(serialPort *serial.Port)
	// Done only when user hits configure button, return true if something was written
	writeConfigFromSettings(serialPort *serial.Port) bool
}

func formatOgnTrackerConfigString() string {
	msg := fmt.Sprintf("$POGNS,Address=0x%s,AddrType=%d,AcftType=%d,Pilot=%s,Reg=%s,TxPower=%d,Hard=STX,Soft=%s",
		globalSettings.OGNAddr, globalSettings.OGNAddrType, globalSettings.OGNAcftType, globalSettings.OGNPilot, globalSettings.OGNReg, globalSettings.OGNTxPower, stratuxVersion[1:])
	msg = appendNmeaChecksum(msg)
	return msg + "\r\n"
}


type OgnTracker struct {
	detected bool
	trackerConfig []string
}

type GxAirCom struct {
	detected bool
	trackerConfig []string
	// If we suspect GX, but don't know for sure, we send exactly one blind config request. If it responds, we know. Otherwise, we stop.
	blindAskedForConfig bool
}

type SoftRF struct {
	detected bool
	settings map[string]string
}

func (tracker *OgnTracker) initNewConnection(serialPort *serial.Port) {
	tracker.detected = false
	tracker.trackerConfig = nil
}

func (tracker *OgnTracker) onNmea(serialPort *serial.Port, nmea []string) bool {
	if nmea[0] == "POGNR" {
		tracker.detected = true
		return true
	}

	if nmea[0] == "POGNS" {
		tracker.trackerConfig = nmea

		// Tracker notified us of restart (crashed?) -> ensure we configure it again
		if len(nmea) == 2 && nmea[1] == "SysStart" {
			tracker.writeInitialConfig(serialPort)
			return true
		}
		// OGN tracker sent us its configuration
		log.Printf("Received OGN Tracker configuration: " + strings.Join(nmea, ","))
		oldAddr := globalSettings.OGNAddr
		for i := 1; i < len(nmea); i++ {
			kv := strings.SplitN(nmea[i], "=", 2);
			if len(kv) < 2 {
				continue
			}

			if kv[0] == "Address" {
				addr, _ :=  strconv.ParseUint(kv[1], 0, 32)
				globalSettings.OGNAddr = strings.ToUpper(fmt.Sprintf("%x", addr))
			} else if kv[0] == "AddrType" {
				addrtype, _ :=  strconv.ParseInt(kv[1], 0, 8)
				globalSettings.OGNAddrType = int(addrtype)
			} else if kv[0] == "AcftType" {
				acfttype, _ :=  strconv.ParseInt(kv[1], 0, 8)
				globalSettings.OGNAcftType = int(acfttype)
			} else if kv[0] == "Pilot" {
				globalSettings.OGNPilot = kv[1]
			} else if kv[0] == "Reg" {
				globalSettings.OGNReg = kv[1]
			} else if kv[0] == "TxPower" {
				pwr, _ := strconv.ParseInt(kv[1], 10, 16)
				globalSettings.OGNTxPower = int(pwr)
			}
		}
		// OGN Tracker can change its address arbitrarily. However, if it does,
		// ownship detection would fail for the old target. Therefore we remove the old one from the traffic list
		if oldAddr != globalSettings.OGNAddr && globalSettings.OGNAddrType == 0 {
			globalStatus.OGNPrevRandomAddr = oldAddr
			oldAddrInt, _ := strconv.ParseUint(oldAddr, 16, 32)
			removeTarget(uint32(oldAddrInt))
			// potentially other address type before
			removeTarget(uint32((1 << 24) | oldAddrInt))
		}
	}
	return false

}
func (tracker *OgnTracker) gpsTimeOffsetPps() time.Duration {
	return 200 * time.Millisecond
}

func (tracker *OgnTracker) getGpsHardwareType() uint {
	return GPS_TYPE_OGNTRACKER
	
}
func (tracker *OgnTracker) isDetected() bool {
	return tracker.detected
}

func (tracker *OgnTracker) isConfigRead() bool {
	return tracker.trackerConfig != nil
}

func (tracker *OgnTracker) writeReadDelay() time.Duration {
	return 1 * time.Second
}

func (tracker *OgnTracker) writeInitialConfig(serialPort *serial.Port) bool {
	serialPort.Write([]byte(appendNmeaChecksum("$POGNS,NavRate=5") + "\r\n"))
	// Configuration for OGN Tracker T-Beam is similar to normal Ublox config
	writeUblox8ConfigCommands(serialPort)
	writeUbloxGenericCommands(5, serialPort)
	return true
}

func (tracker *OgnTracker) requestTrackerConfig(serialPort *serial.Port) {
	serialPort.Write([]byte(appendNmeaChecksum("$POGNS") + "\r\n"))
}

func (tracker *OgnTracker) writeConfigFromSettings(serialPort *serial.Port) bool {
	cfg := formatOgnTrackerConfigString()
	log.Printf("Configuring OGN Tracker: " + cfg)

	serialPort.Write([]byte(cfg))
	tracker.requestTrackerConfig(serialPort) // re-read settings from tracker
	return true
}

func (tracker *GxAirCom) initNewConnection(serialPort *serial.Port) {
	tracker.detected = false
	tracker.trackerConfig = nil
	tracker.blindAskedForConfig = false
}


func (tracker *GxAirCom) onNmea(serialPort *serial.Port, nmea []string) bool {
    // Only sent by GxAirCom tracker. We use this to detect that GxAirCom tracker is connected and configure it as needed
    if (nmea[0] == "PFLAV" && nmea[4] == "GXAircom") {
		tracker.detected = true
        return true
    }
	if nmea[0] == "LK8EX1" && !tracker.blindAskedForConfig {
		// Indicates potentially GxAirCom.. send a blind tracker config request. If it responds, we know for sure
		tracker.blindAskedForConfig = true
		tracker.requestTrackerConfig(serialPort)
		return true
	}

	if nmea[0] == "PGXCF" && nmea[1] == "1" {
		// $PGXCF,<version>,<Output Serial>,<eMode>,<eOutputVario>,<output Fanet>,<output GPS>,<output FLARM>,<customGPSConfig>,<Aircraft Type (hex)>,<Address Type>,<Address (hex)>,<Pilot Name> 
		//  0      1         2               3       4              5            6              7                 8                     9              10              11             12
		log.Printf("Received GxAirCom Tracker configuration: " + strings.Join(nmea, ","))
		tracker.detected = true
		tracker.trackerConfig = nmea

		GXAcftType,_ := strconv.ParseInt(nmea[9], 16, 0)
		acftType := mapAircraftType(typeMappingOgn2Gx, false, int(GXAcftType))
		globalSettings.OGNAcftType = acftType

		GXAddrType,_ := strconv.Atoi(nmea[10])
		globalSettings.OGNAddrType = GXAddrType
		globalSettings.OGNAddr = nmea[11]
		globalSettings.OGNPilot = nmea[12]

		return true
	}

	return false

}
func (tracker *GxAirCom) gpsTimeOffsetPps() time.Duration {
	return 130 * time.Millisecond
}

func (tracker *GxAirCom) getGpsHardwareType() uint {
	return GPS_TYPE_GXAIRCOM
	
}
func (tracker *GxAirCom) isDetected() bool {
	return tracker.detected
}

func (tracker *GxAirCom) isConfigRead() bool {
	return tracker.trackerConfig != nil
}

func (tracker *GxAirCom) writeReadDelay() time.Duration {
	return 15 * time.Second
}

func (tracker *GxAirCom) writeInitialConfig(serialPort *serial.Port) bool {
	if (tracker.trackerConfig[2] != "1" || tracker.trackerConfig[5] == "0" || tracker.trackerConfig[6] == "0" || tracker.trackerConfig[7] == "0" || tracker.trackerConfig[8] == "0") {
		tracker.writeConfigFromSettings(serialPort) // force correct settings
		return true
	} else {
		log.Printf("GxAirCom tracker configuration ok!")
		return false
	}
}

func (tracker *GxAirCom) requestTrackerConfig(serialPort *serial.Port) {
	serialPort.Write([]byte(appendNmeaChecksum("$PGXCF,?") + "\r\n")) // Request configuration
}

func (tracker *GxAirCom) writeConfigFromSettings(serialPort *serial.Port) bool {
	// GX Addr type: 1=ICAO 2=FLARM
	addrType := globalSettings.OGNAddrType
	if addrType != 1 {
		addrType = 2 // all other address types to FLARM = non-ICAO
	}

	acftType := mapAircraftType(typeMappingOgn2Gx, true, globalSettings.OGNAcftType)


	// $PGXCF,<version>,<Output Serial>,<eMode>,<eOutputVario>,<output Fanet>,<output GPS>,<output FLARM>,<customGPSConfig>,<Aircraft Type (hex)>,<Address Type>,<Address (hex)>,<Pilot Name> 
	//  0      1         2               3              4              5            6              7                 8                     9              10              11      12
	requiredSentence := fmt.Sprintf("$PGXCF,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s",
		1,  // PGXCF Version
		1,  // Serial out
		0,  // Airmode
		1,  // Vario disabled // 0=noVario, 1= LK8EX1, 2=LXPW, we use it to try to identify GxAirCom
		1,  // Fanet
		1,  // GPS
		1,  // Flarm
		1,  // Stratux NMEA
		acftType,
		addrType,
		globalSettings.OGNAddr,
		globalSettings.OGNPilot)

	fullSentence := appendNmeaChecksum(requiredSentence)
	log.Printf("Configuring GxAirCom Tracker with: " + fullSentence)
	serialPort.Write([]byte(fullSentence + "\r\n")) // Set configuration
	return true
}

func (tracker *SoftRF) initNewConnection(serialPort *serial.Port) {
	tracker.detected = false
	tracker.settings = make(map[string]string)
}


func (tracker *SoftRF) onNmea(serialPort *serial.Port, nmea []string) bool {
	if nmea[0] == "PSRFH" {
		if tracker.detected && !tracker.isConfigRead() {
			// sometimes SoftRF doesn't seem to respond to settings request if we are too early after boot.. retry if we were already detected before but still don't have a config
			tracker.requestTrackerConfig(serialPort)
		}
		tracker.detected = true
		return true
	}

	// See output of $PSRFS,0,?*4B
	if nmea[0] == "PSRFS" {
		key, value := nmea[2], nmea[3]
		if value == "?" {
			return true // ignore query echoes
		}
		log.Printf("Received SoftRF config %s=%s", key, value)
		tracker.settings[key] = value
		if key == "acft_type" {
			acType, _ := strconv.Atoi(value)
			acType = mapAircraftType(typeMappingOgn2SoftRF, false, acType)
			globalSettings.OGNAcftType = acType
		} else if key == "id_method" {
			globalSettings.OGNAddrType, _ = strconv.Atoi(value)
		} else if key == "aircraft_id" {
			globalSettings.OGNAddr = value
		} else if key == "protocol" {
			globalSettings.SoftRFProtocol, _ = strconv.Atoi(value)
		} else if key == "altprotocol" {
			globalSettings.SoftRFAltProtocol, _ = strconv.Atoi(value)
		} else if key == "band" {
			globalSettings.SoftRFBand, _ = strconv.Atoi(value)
		} else if key == "alarm" {
			globalSettings.SoftRFAlarm, _ = strconv.Atoi(value)
		} else if key == "relay" {
			globalSettings.SoftRFRelay, _ = strconv.Atoi(value)
		} else if key == "tx_power" {
			globalSettings.SoftRFTxPower, _ = strconv.Atoi(value)
		} else if key == "stealth" {
			globalSettings.SoftRFStealth = value == "1"
		} else if key == "no_track" {
			globalSettings.SoftRFNoTrack = value == "1"
		}
		return true
	}

	return false

}
func (tracker *SoftRF) gpsTimeOffsetPps() time.Duration {
	return 200 * time.Millisecond
}

func (tracker *SoftRF) getGpsHardwareType() uint {
	return GPS_TYPE_SOFTRF
	
}
func (tracker *SoftRF) isDetected() bool {
	return tracker.detected
}

func (tracker *SoftRF) isConfigRead() bool {
	return len(tracker.settings) >= 12 // need identity (3) + nmea gates (2) + rf settings (protocol, altprotocol, band, tx_power, alarm, relay, stealth)
}

func (tracker *SoftRF) writeReadDelay() time.Duration {
	return 15 * time.Second // SoftRF seems to take a while until it responds to config read after boot..?
}

func (tracker *SoftRF) writeInitialConfig(serialPort *serial.Port) bool {
	if !tracker.isConfigRead() {
		return false
	}
	changed := false
	if tracker.settings["nmea_g"] != "0F" {
		msg := appendNmeaChecksum("$PSRFS,0,nmea_g,0F") + "\r\n"
		log.Printf("Configure SoftRF: %s", msg)
		serialPort.Write([]byte(msg))
		changed = true
	}
	if tracker.settings["nmea2_g"] != "0F" {
		msg := appendNmeaChecksum("$PSRFS,0,nmea2_g,0F") + "\r\n"
		log.Printf("Configure SoftRF: %s", msg)
		serialPort.Write([]byte(msg))
		changed = true
	}
	if changed {
		serialPort.Write([]byte(appendNmeaChecksum("$PSRFC,SAV") + "\r\n"))
	}
	return false
}

func (tracker *SoftRF) requestTrackerConfig(serialPort *serial.Port) {
	keys := []string{
		"nmea_g", "nmea2_g",
		"acft_type", "aircraft_id", "id_method",
		"protocol", "altprotocol", "band", "alarm", "relay",
		"tx_power", "stealth", "no_track",
	}
	for _, key := range keys {
		serialPort.Write([]byte(appendNmeaChecksum("$PSRFS,0," + key + ",?") + "\r\n"))
	}
}

func (tracker *SoftRF) softRFSettingChanged(key string, desired string) bool {
	if s, ok := tracker.settings[key]; !ok || s != desired {
		return true
	}
	return false
}

func (tracker *SoftRF) writeConfigFromSettings(serialPort *serial.Port) bool {
	if !tracker.isConfigRead() {
		return false
	}

	acType := strconv.Itoa(mapAircraftType(typeMappingOgn2SoftRF, true, globalSettings.OGNAcftType))
	addrType := strconv.Itoa(globalSettings.OGNAddrType)
	addr := globalSettings.OGNAddr

	var messages []string

	if tracker.softRFSettingChanged("acft_type", acType) {
		messages = append(messages, appendNmeaChecksum("$PSRFS,0,acft_type," + acType) + "\r\n")
	}
	if tracker.softRFSettingChanged("id_method", addrType) {
		messages = append(messages, appendNmeaChecksum("$PSRFS,0,id_method," + addrType) + "\r\n")
	}
	if tracker.softRFSettingChanged("aircraft_id", addr) {
		messages = append(messages, appendNmeaChecksum("$PSRFS,0,aircraft_id," + addr) + "\r\n")
	}

	// SoftRF-specific settings: only send if they have been read from the device (>= 0 or -1 means unread)
	if globalSettings.SoftRFProtocol > 0 {
		desired := strconv.Itoa(globalSettings.SoftRFProtocol)
		if tracker.softRFSettingChanged("protocol", desired) {
			messages = append(messages, appendNmeaChecksum("$PSRFS,0,protocol," + desired) + "\r\n")
		}
	}
	if globalSettings.SoftRFAltProtocol >= 0 {
		desired := strconv.Itoa(globalSettings.SoftRFAltProtocol)
		if tracker.softRFSettingChanged("altprotocol", desired) {
			messages = append(messages, appendNmeaChecksum("$PSRFS,0,altprotocol," + desired) + "\r\n")
		}
	}
	if globalSettings.SoftRFBand > 0 {
		desired := strconv.Itoa(globalSettings.SoftRFBand)
		if tracker.softRFSettingChanged("band", desired) {
			messages = append(messages, appendNmeaChecksum("$PSRFS,0,band," + desired) + "\r\n")
		}
	}
	if globalSettings.SoftRFAlarm >= 0 {
		desired := strconv.Itoa(globalSettings.SoftRFAlarm)
		if tracker.softRFSettingChanged("alarm", desired) {
			messages = append(messages, appendNmeaChecksum("$PSRFS,0,alarm," + desired) + "\r\n")
		}
	}
	if globalSettings.SoftRFRelay >= 0 {
		desired := strconv.Itoa(globalSettings.SoftRFRelay)
		if tracker.softRFSettingChanged("relay", desired) {
			messages = append(messages, appendNmeaChecksum("$PSRFS,0,relay," + desired) + "\r\n")
		}
	}
	if globalSettings.SoftRFTxPower >= 0 {
		desired := strconv.Itoa(globalSettings.SoftRFTxPower)
		if tracker.softRFSettingChanged("tx_power", desired) {
			messages = append(messages, appendNmeaChecksum("$PSRFS,0,tx_power," + desired) + "\r\n")
		}
	}

	// Boolean settings: always send if different from device
	stealthDesired := "0"
	if globalSettings.SoftRFStealth {
		stealthDesired = "1"
	}
	if tracker.softRFSettingChanged("stealth", stealthDesired) {
		messages = append(messages, appendNmeaChecksum("$PSRFS,0,stealth," + stealthDesired) + "\r\n")
	}

	noTrackDesired := "0"
	if globalSettings.SoftRFNoTrack {
		noTrackDesired = "1"
	}
	if tracker.softRFSettingChanged("no_track", noTrackDesired) {
		messages = append(messages, appendNmeaChecksum("$PSRFS,0,no_track," + noTrackDesired) + "\r\n")
	}

	for _, msg := range messages {
		log.Printf("Configure SoftRF: %s", msg)
		serialPort.Write([]byte(msg))
	}

	if len(messages) > 0 {
		serialPort.Write([]byte(appendNmeaChecksum("$PSRFC,SAV") + "\r\n")) // Save and reboot
	}

	return len(messages) > 0
}










