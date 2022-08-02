/*
	Copyright (c) 2015-2016 Christopher Young
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	gps.go: GPS functions, GPS init, AHRS status messages, other external sensor monitoring.
*/

package main

import (
	"fmt"
	"log"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/b3nn0/stratux/v2/common"
	"github.com/b3nn0/stratux/v2/gps"
	cmap "github.com/orcaman/concurrent-map"
)

// Array of NMEA lines we would always want to process from any attached GPS device
// This should never contain any NMEA codes for GPS Location data
var ALWAYS_PROCESS_NMEAS = []string{"PFLAU", "PFLAA", "POGNS", "POGNR", "POGNB", "PSOFT"}

const GPS_FIX_TIME = 5000 * time.Millisecond              // Time we expect a GPS satelite to have a valid fix
const GPS_TIME_SOURCE = 2000 * time.Millisecond           // Time we expect the GPS to be a valid source before we reconsider other GPS location sources
const GPS_VALID_TIME = 3000 * time.Millisecond            // TIme we consider a GPS data source to be valid
const GPS_CONSIDERING_CONNECTED = 5000 * time.Millisecond // Time we expect a GPS as beeing connected to stratux
const FIX_QUALITY_3DGPS = 1                               // 3DGPS
const FIX_QUALITY_AGPS = 2                                // SBAS/WAAS

type SatelliteInfo struct {
	SatelliteNMEA    uint8     // NMEA ID of the satellite. 1-32 is GPS, 33-54 is SBAS, 65-88 is Glonass.
	SatelliteID      string    // Formatted code indicating source and PRN code. e.g. S138==WAAS satellite 138, G2==GPS satellites 2
	Elevation        int16     // Angle above local horizon, -xx to +90
	Azimuth          int16     // Bearing (degrees true), 0-359
	Signal           int8      // Signal strength, 0 - 99; -99 indicates no reception
	Type             uint8     // Type of satellite (GPS, GLONASS, Galileo, SBAS)
	TimeLastSolution time.Time // Time (system ticker) a solution was last calculated using this satellite
	TimeLastSeen     time.Time // Time (system ticker) a signal was last received from this satellite
	TimeLastTracked  time.Time // Time (system ticker) this satellite was tracked (almanac data)
	InSolution       bool      // True if satellite is used in the position solution (reported by GSA message or PUBX,03)
}

type SituationData struct {
	// From GPS.
	muGPS                       *sync.Mutex
	muGPSPerformance            *sync.Mutex
	muSatellite                 *sync.Mutex
	GPSLastFixSinceMidnightUTC  float32
	GPSLatitude                 float32
	GPSLongitude                float32
	GPSFixQuality               uint8
	GPSHeightAboveEllipsoid     float32 // GPS height above WGS84 ellipsoid, ft. This is specified by the GDL90 protocol, but most EFBs use MSL altitude instead. HAE is about 70-100 ft below GPS MSL altitude over most of the US.
	GPSGeoidSep                 float32 // geoid separation, ft, MSL minus HAE (used in altitude calculation)
	GPSSatellites               uint16  // satellites used in solution
	GPSSatellitesTracked        uint16  // satellites tracked (almanac data received)
	GPSSatellitesSeen           uint16  // satellites seen (signal received)
	GPSHorizontalAccuracy       float32 // 95% confidence for horizontal position, meters.
	GPSHDop                     float32 // GPS HDOP
	GPSVDop                     float32 // GPS VDOP
	GPSNACp                     uint8   // NACp categories are defined in AC 20-165A
	GPSAltitudeMSL              float32 // Feet MSL
	GPSVerticalAccuracy         float32 // 95% confidence for vertical position, meters
	GPSVerticalSpeed            float32 // GPS vertical velocity, feet per second
	GPSLastFixLocalTime         time.Time
	GPSTrueCourse               float32
	GPSTurnRate                 float64 // calculated GPS rate of turn, degrees per second
	GPSGroundSpeed              float64
	GPSLastGroundTrackTime      time.Time
	GPSTime                     time.Time
	GPSLastGPSTimeStratuxTime   time.Time // stratuxClock time since last GPS time received.
	GPSLastValidNMEAMessageTime time.Time // time valid NMEA message last seen
	GPSLastValidNMEAMessage     string    // last NMEA message processed.
	GPSPositionSampleRate       float64   // calculated sample rate of GPS positions

	// From pressure sensor.
	muBaro                  *sync.Mutex
	BaroTemperature         float32
	BaroPressureAltitude    float32
	BaroVerticalSpeed       float32
	BaroLastMeasurementTime time.Time
	BaroSourceType          uint8

	// From AHRS source.
	muAttitude           *sync.Mutex
	AHRSPitch            float64
	AHRSRoll             float64
	AHRSGyroHeading      float64
	AHRSMagHeading       float64
	AHRSSlipSkid         float64
	AHRSTurnRate         float64
	AHRSGLoad            float64
	AHRSGLoadMin         float64
	AHRSGLoadMax         float64
	AHRSLastAttitudeTime time.Time
	AHRSStatus           uint8
}

/*
myGPSPerfStats used to track short-term position / velocity trends, used to feed dynamic AHRS model. Use floats for better resolution of calculated data.
*/
type gpsPerfStats struct {
	stratuxTime   uint64  // time since Stratux start, msec
	nmeaTime      float32 // timestamp from NMEA message
	msgType       string  // NMEA message type
	gsf           float32 // knots
	coursef       float32 // true course [degrees]
	alt           float32 // gps altitude, ft msl
	vv            float32 // vertical velocity, ft/sec
	gpsTurnRate   float64 // calculated turn rate, deg/sec. Right turn is positive.
	gpsPitch      float64 // estimated pitch angle, deg. Calculated from gps ground speed and VV. Equal to flight path angle.
	gpsRoll       float64 // estimated roll angle from turn rate and groundspeed, deg. Assumes airplane in coordinated turns.
	gpsLoadFactor float64 // estimated load factor from turn rate and groundspeed, "gee". Assumes airplane in coordinated turns.
	//TODO: valid/invalid flag.
}

var gpsPerf gpsPerfStats
var myGPSPerfStats []gpsPerfStats
var gpsTimeOffsetPpsMs = 100.0 * time.Millisecond

var Satellites map[string]SatelliteInfo

var ognTrackerConfigured = false

type GPSDeviceStatus struct {
	gpsSource      uint16
	gpsFixQuality  uint8
	gpsLastSeen    time.Time
	gpsLastGoodFix time.Time
	gpsCRCErrors   uint64
}

type GPSDeviceManager struct {
	gpsDeviceStatus     map[string]GPSDeviceStatus // Map of device status when they are sending us message
	settingsCopy        settings                   // Copy of the global Settings to help decide if some varas are changed
	currentGPSName      string                     // name of the current GPS device we receive location data from
	qh                  *common.QuitHelper         // Helper to quick various routines during reconfiguratios
	m                   sync.Mutex                 // Mutex to lock gpsDeviceStatus
	rxMessageCh         chan gps.RXMessage         // Channel used by all GPS devices to receive NMEA data from
	discoveredDevicesCh chan gps.DiscoveredDevice  // Channel used to send information about discovered devices
	txMessageCh         chan gps.TXMessage         // Channel used to send a any device a message

	ognTrackerConfigured bool
	systemTimeSetter	*gps.OSTimeSetter
	discoveredDevices cmap.ConcurrentMap
}

func NewGPSDeviceManager() GPSDeviceManager {

	return GPSDeviceManager{
		gpsDeviceStatus:      make(map[string]GPSDeviceStatus),
		settingsCopy:         globalSettings,
		currentGPSName:       "",
		qh:                   common.NewQuitHelper(),
		m:                    sync.Mutex{},
		rxMessageCh:          make(chan gps.RXMessage, 20),
		txMessageCh:          make(chan gps.TXMessage, 20),
		discoveredDevicesCh:  make(chan gps.DiscoveredDevice, 20),
		ognTrackerConfigured: false,
		systemTimeSetter:     gps.NewOSTimeSetter(),
		discoveredDevices:    cmap.New(),
	}
}

/*
	registerSituationUpdate().
	 Called whenever there is a change in mySituation.
*/
func registerSituationUpdate() {
	logSituation()
	situationUpdate.SendJSON(mySituation)
}

/*
	updateConstellation(): Periodic cleanup and statistics calculation for 'Satellites'
		data structure. Calling functions must protect this in a mySituation.muSatellite.

*/

func updateConstellation() (sats uint16, tracked uint16, seen uint16) {
	// TODO: RVT: THis function updates and returns
	mySituation.muSatellite.Lock()
	defer mySituation.muSatellite.Unlock()
	for svStr, thisSatellite := range Satellites {
		if stratuxClock.Since(thisSatellite.TimeLastTracked) > 10*time.Second { // remove stale satellites if they haven't been tracked for 10 seconds
			delete(Satellites, svStr)
		} else { // satellite almanac data is "fresh" even if it isn't being received.
			tracked++
			if thisSatellite.Signal > 0 {
				seen++
			}
			if stratuxClock.Since(thisSatellite.TimeLastSolution) > 5*time.Second {
				thisSatellite.InSolution = false
				Satellites[svStr] = thisSatellite
			}
			if thisSatellite.InSolution { // TESTING: Determine "In solution" from structure (fix for multi-GNSS overflow)
				sats++
			}
			// do any other calculations needed for this satellite
		}
	}
	return
}

/**
  Update satelite information from GPS data
  This should be used from GNGSA,GPGSA,GLGSA,GAGSA,GBGSA sentences only
*/
func updateSatellites(x []string) {
	mySituation.muSatellite.Lock()
	defer mySituation.muSatellite.Unlock()
	for _, svtxt := range x[3:15] {

		sv, svType, svStr, err := getsvTypesvStr(svtxt)

		if err == nil {
			var thisSatellite SatelliteInfo

			// Retrieve previous information on this satellite code.
			if val, ok := Satellites[svStr]; ok { // if we've already seen this satellite identifier, copy it in to do updates
				thisSatellite = val
				//log.Printf("Satellite %s already seen. Retrieving from 'Satellites'.\n", svStr)
			} else { // this satellite isn't in the Satellites data structure, so create it
				thisSatellite.SatelliteID = svStr
				thisSatellite.SatelliteNMEA = uint8(sv)
				thisSatellite.Type = uint8(svType)
				//log.Printf("Creating new satellite %s from GSA message\n", svStr) // DEBUG
			}
			thisSatellite.InSolution = true
			thisSatellite.TimeLastSolution = stratuxClock.Time
			thisSatellite.TimeLastSeen = stratuxClock.Time    // implied, since this satellite is used in the position solution
			thisSatellite.TimeLastTracked = stratuxClock.Time // implied, since this satellite is used in the position solution

			Satellites[thisSatellite.SatelliteID] = thisSatellite // Update constellation with this satellite
		}
	}
}

/**
  Update satelite information from GPS data
  This should be used from GPGSV,GLGSV,GAGSV,GBGSV sentences only
*/
func updateSatellitesInView(x []string) {
	mySituation.muSatellite.Lock()
	defer mySituation.muSatellite.Unlock()

	msgNum, _ := strconv.Atoi(x[2]) // TODO: RVT is this not x[1] ??
	msgIndex, _ := strconv.Atoi(x[2])

	// field 3 = number of GPS satellites tracked
	/* Is this redundant if parsing from full constellation?
	satTracked, err := strconv.Atoi(x[3])
	if err != nil {
		return false
	}
	*/

	//mySituation.GPSSatellitesTracked = uint16(satTracked) // Replaced with parsing of 'Satellites' data structure

	// field 4-7 = repeating block with satellite id, elevation, azimuth, and signal strengh (Cno)

	lenGSV := len(x)
	satsThisMsg := (lenGSV - 4) / 4

	if globalSettings.DEBUG {
		log.Printf("%s message [%d of %d] is %v fields long and describes %v satellites\n", x[0], msgIndex, msgNum, lenGSV, satsThisMsg)
	}

	var elev, az, cno int

	for i := 0; i < satsThisMsg; i++ {

		sv, svType, svStr, err := getsvTypesvStr(x[4+4*i])

		if err == nil {

			var thisSatellite SatelliteInfo

			// Retrieve previous information on this satellite code.
			if val, ok := Satellites[svStr]; ok { // if we've already seen this satellite identifier, copy it in to do updates
				thisSatellite = val
				//log.Printf("Satellite %s already seen. Retrieving from 'Satellites'.\n", svStr) // DEBUG
			} else { // this satellite isn't in the Satellites data structure, so create it new
				thisSatellite.SatelliteID = svStr
				thisSatellite.SatelliteNMEA = uint8(sv)
				thisSatellite.Type = uint8(svType)
				//log.Printf("Creating new satellite %s\n", svStr) // DEBUG
			}
			thisSatellite.TimeLastTracked = stratuxClock.Time

			elev, err = strconv.Atoi(x[5+4*i]) // elevation
			if err != nil {                    // some firmwares leave this blank if there's no position fix. Represent as -999.
				elev = -999
			}
			thisSatellite.Elevation = int16(elev)

			az, err = strconv.Atoi(x[6+4*i]) // azimuth
			if err != nil {                  // UBX allows tracking up to 5(?) degrees below horizon. Some firmwares leave this blank if no position fix. Represent invalid as -999.
				az = -999
			}
			thisSatellite.Azimuth = int16(az)

			cno, err = strconv.Atoi(x[7+4*i]) // signal
			if err != nil {                   // will be blank if satellite isn't being received. Represent as -99.
				cno = -99
				thisSatellite.InSolution = false // resets the "InSolution" status if the satellite disappears out of solution due to no signal. FIXME
				//log.Printf("Satellite %s is no longer in solution due to cno parse error - GSV\n", svStr) // DEBUG
			} else if cno > 0 {
				thisSatellite.TimeLastSeen = stratuxClock.Time // Is this needed?
			}
			if cno > 127 { // make sure strong signals don't overflow. Normal range is 0-99 so it shouldn't, but take no chances.
				cno = 127
			}
			thisSatellite.Signal = int8(cno)

			// hack workaround for GSA 12-sv limitation... if this is a SBAS satellite, we have a SBAS solution, and signal is greater than some arbitrary threshold, set InSolution
			// drawback is this will show all tracked SBAS satellites as being in solution.
			if thisSatellite.Type == common.SAT_TYPE_SBAS {
				if mySituation.GPSFixQuality == FIX_QUALITY_AGPS {
					if thisSatellite.Signal > 16 {
						thisSatellite.InSolution = true
						thisSatellite.TimeLastSolution = stratuxClock.Time
					}
				} else { // quality == 0 or 1
					thisSatellite.InSolution = false
					//log.Printf("WAAS satellite %s is marked as out of solution GSV\n", svStr) // DEBUG
				}
			}

			if globalSettings.DEBUG {
				inSolnStr := " "
				if thisSatellite.InSolution {
					inSolnStr = "+"
				}
				log.Printf("GSV: Satellite %s%s at index %d. Type = %d, NMEA-ID = %d, Elev = %d, Azimuth = %d, Cno = %d\n", inSolnStr, svStr, i, svType, sv, elev, az, cno) // remove later?
			}

			Satellites[thisSatellite.SatelliteID] = thisSatellite // Update constellation with this satellite
		}
	}
}

func updateGPSPerfmStat(thisGpsPerf gpsPerfStats) {
	mySituation.muGPSPerformance.Lock()
	defer mySituation.muGPSPerformance.Unlock()
	myGPSPerfStats = append(myGPSPerfStats, thisGpsPerf)
	lenGPSPerfStats := len(myGPSPerfStats)
	//	log.Printf("GPSPerf array has %n elements. Contents are: %v\n",lenGPSPerfStats,myGPSPerfStats)
	if lenGPSPerfStats > 299 { //30 seconds @ 10 Hz for UBX, 30 seconds @ 5 Hz for MTK or SIRF with 2x messages per 200 ms)
		myGPSPerfStats = myGPSPerfStats[(lenGPSPerfStats - 299):] // remove the first n entries if more than 300 in the slice
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Process a NMEA line and update strauc
//////////////////////////////////////////////////////////////////////////////////////////////////////


func (s *GPSDeviceManager) processNMEALine(l string, name string, deviceDiscovery gps.DiscoveredDevice) (sentenceUsed bool) {
	deviceDiscovery.HasTXChannel = false
	mySituation.muGPS.Lock()
	defer func() {
		mySituation.GPSLastValidNMEAMessageTime = stratuxClock.Time
		mySituation.GPSLastValidNMEAMessage = l
		mySituation.muGPS.Unlock()
	}()
	// Simulate in-flight moving GPS, useful in combination with demo traffic in gen_gdl90.go
	/*defer func() {
		tmpSituation := mySituation
		if strings.Contains(l, "GGA,") || strings.Contains(l, "RMC,") {
			tmpSituation.GPSLatitude += float32(stratuxClock.Milliseconds) / 1000.0 / 60.0 / 30.0
		}

		tmpSituation.GPSTrueCourse = 0
		tmpSituation.GPSGroundSpeed = 110
		tmpSituation.GPSAltitudeMSL = 5000
		tmpSituation.GPSHeightAboveEllipsoid = 5000
		tmpSituation.BaroPressureAltitude = 4800
		mySituation = tmpSituation
	}()*/

	// Local variables for GPS attitude estimation
	thisGpsPerf := gpsPerf                              // write to myGPSPerfStats at end of function IFF
	thisGpsPerf.coursef = -999.9                        // default value of -999.9 indicates invalid heading to regression calculation
	thisGpsPerf.stratuxTime = stratuxClock.Milliseconds // used for gross indexing

	x := strings.Split(l, ",")

	// ############################################# GNVTG GNVTG #############################################
	if (x[0] == "GNVTG") || (x[0] == "GPVTG") { // Ground track information.
		// RTV: Verified
		data, err := parseNMEALine_GNVTG_GPVTG(x, &mySituation)
		if err == nil {
			mySituation = data
		}
		return err == nil
		// ############################################# GNGGA GPGGA #############################################
	} else if (x[0] == "GNGGA") || (x[0] == "GPGGA") { // Position fix.
		// RTV: Verified
		data, err := parseNMEALine_GNGGA_GPGGA(x, &mySituation)
		if err == nil {
			mySituation = data

			// AT65 'ticks' the seconds, based on this we set the date/tome in statux.
			// Do other GPSes also this?
			t := mySituation.GPSLastFixSinceMidnightUTC
			hh := int(t / 3600)
			mm := int(t-float32(hh)*3600) / 60
			ss := int(t - float32(hh)*3600 - float32(mm)*60)
			t1 := time.Date(
				mySituation.GPSTime.Year(),
				mySituation.GPSTime.Month(),
				mySituation.GPSTime.Day(),
				hh, mm, ss, 0, mySituation.GPSTime.Location())

			gpsTime := t1.Add(deviceDiscovery.GpsTimeOffsetPpsMs)
			mySituation.GPSTime = gpsTime
			thisGpsPerf.nmeaTime = mySituation.GPSLastFixSinceMidnightUTC
			thisGpsPerf.alt = float32(mySituation.GPSAltitudeMSL)
			thisGpsPerf.msgType = x[0]
			updateGPSPerfmStat(thisGpsPerf)

			if (globalStatus.GPS_detected_type & 0x0F) == common.GPS_TYPE_SOFTRF_AT65 {
				s.systemTimeSetter.SetTime(gpsTime)
				stratuxClock.SetRealTimeReference(gpsTime)
			}
		}
		return err == nil
		// ############################################# GNRMC GPRMC #############################################
	} else if (x[0] == "GNRMC") || (x[0] == "GPRMC") { // Recommended Minimum data.
		// RVT: Verified
		data, err := parseNMEALine_GNRMC_GPRMC(x, &mySituation)
		if err == nil {
			previousSituation := mySituation
			mySituation = data

			// Unset course if the GS was low
			if mySituation.GPSGroundSpeed > 3 {
				thisGpsPerf.coursef = float32(previousSituation.GPSTrueCourse)
			} else {
				mySituation.GPSTrueCourse = previousSituation.GPSTrueCourse
				thisGpsPerf.coursef = -999.9
			}
			thisGpsPerf.gsf = float32(mySituation.GPSGroundSpeed)
			thisGpsPerf.msgType = x[0]
			thisGpsPerf.nmeaTime = mySituation.GPSLastFixSinceMidnightUTC
			mySituation.GPSTime = mySituation.GPSTime.Add(deviceDiscovery.GpsTimeOffsetPpsMs)
			// We need to set AT65 type GPS in GNGGA because for this GPS that's the start of a real second
			if len(x[9]) == 6 && (globalStatus.GPS_detected_type&0x0F) != common.GPS_TYPE_SOFTRF_AT65 {
				s.systemTimeSetter.SetTime(mySituation.GPSTime)
				stratuxClock.SetRealTimeReference(mySituation.GPSTime)
			}

			updateGPSPerfmStat(thisGpsPerf)
			setDataLogTimeWithGPS(mySituation)
		}
		return err == nil
		// ############################################# GNGSA GPGSA GLGSA GAGSA GBGSA #############################################
		// TODO: RVT : In original GPS code we only do GNGSA and GLGSA, why do we now do more??
	} else if (x[0] == "GNGSA") || (x[0] == "GPGSA") || (x[0] == "GLGSA") || (x[0] == "GAGSA") || (x[0] == "GBGSA") { // Satellite data.
		// RVT: Verified
		data, err := parseNMEALine_GNGSA_GPGSA_GLGSA_GAGSA_GBGSA(x, &mySituation)
		if err == nil {
			if data.GPSFixQuality == FIX_QUALITY_AGPS { // Rough 95% confidence estimate for SBAS solution
				if globalStatus.GPS_detected_type == common.GPS_TYPE_UBX9 {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 3.0) // ublox 9
				} else {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 4.0) // ublox 6/7/8
				}
			} else { // Rough 95% confidence estimate non-SBAS solution
				if globalStatus.GPS_detected_type == common.GPS_TYPE_UBX9 {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 4.0) // ublox 9
				} else {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 5.0) // ublox 6/7/8
				}
			}
			data.GPSVerticalAccuracy = data.GPSVDop * 5.0 // Rough 95% confidence estimate
			data.GPSNACp = calculateNACp(data.GPSHorizontalAccuracy)

			sats, tracked, seen := updateConstellation()
			data.GPSSatellites = sats
			data.GPSSatellitesTracked = tracked
			data.GPSSatellitesSeen = seen

			mySituation = data

			updateSatellites(x)
		}
		return err == nil
		// ############################################# GPGSV GLGSV GAGSV GBGSV #############################################
	} else if (x[0] == "GPGSV") || (x[0] == "GLGSV") || (x[0] == "GAGSV") || (x[0] == "GBGSV") { // GPS + SBAS or GLONASS or Galileo or Beidou satellites in view message.

		// RVT: Verified
		data, err := parseNMEALine_GPGSV_GLGSV_GAGSV_GBGSV(x, &mySituation)
		if err == nil {

			sats, tracked, seen := updateConstellation()
			data.GPSSatellites = sats
			data.GPSSatellitesTracked = tracked
			data.GPSSatellitesSeen = seen

			mySituation = data

			updateSatellitesInView(x)
		}
		return err == nil

		// ############################################# POGNB #############################################
	} else if x[0] == "POGNB" {
		// RTV: Verified
		data, err := parseNMEALine_POGNB(x, &mySituation)
		if err == nil {
			mySituation.muBaro.Lock()
			mySituation.BaroPressureAltitude = data.BaroPressureAltitude
			mySituation.BaroVerticalSpeed = data.BaroVerticalSpeed
			mySituation.BaroLastMeasurementTime = data.BaroLastMeasurementTime
			mySituation.BaroSourceType = data.BaroSourceType
			mySituation.muBaro.Unlock()
		}
		return err == nil
		// ############################################# POGNB #############################################
	} else if x[0] == "PSOFT" {
		// When PSOFT is send, we know the device is a SOFTRF and we can take special care of setting date/time
		data, err := parseNMEALine_PSOFT(x, &mySituation)
		if err == nil {
			mySituation = data

			if x[1] == "AT65" {
				deviceDiscovery.GpsDetectedType = common.GPS_TYPE_SOFTRF_AT65
				deviceDiscovery.IsTypeUpgrade = true
				s.discoveredDevicesCh <- deviceDiscovery
			}
		}
		return err == nil
		// ############################################# POGNR #############################################
	} else if x[0] == "POGNR" {
		// RVT: Verified
		// Only sent by OGN tracker. We use this to detect that OGN tracker is connected and configure it as needed
		if !s.ognTrackerConfigured {
			s.ognTrackerConfigured = true
			s.requestOgnTrackerConfiguration(name)
		}

		return true
		// ############################################# POGNS #############################################
	} else if x[0] == "POGNS" {
		// RVT: Verified
		// Tracker notified us of restart (crashed?) -> ensure we configure it again
		if len(x) == 2 && x[1] == "SysStart" {
			s.ognTrackerConfigured = false
			return true
		}
		// OGN tracker sent us its configuration
		log.Printf("Received OGN Tracker configuration: %s", strings.Join(x, ","))
		oldAddr := globalSettings.OGNAddr
		for i := 1; i < len(x); i++ {
			kv := strings.SplitN(x[i], "=", 2)
			if len(kv) < 2 {
				continue
			}

			if kv[0] == "Address" {
				addr, _ := strconv.ParseUint(kv[1], 0, 32)
				globalSettings.OGNAddr = strings.ToUpper(fmt.Sprintf("%x", addr))
			} else if kv[0] == "AddrType" {
				addrtype, _ := strconv.ParseInt(kv[1], 0, 8)
				globalSettings.OGNAddrType = int(addrtype)
			} else if kv[0] == "AcftType" {
				acfttype, _ := strconv.ParseInt(kv[1], 0, 8)
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
		return true
		// ############################################# PGRMZ #############################################
	} else if x[0] == "PGRMZ" &&
		((globalStatus.GPS_detected_type&0x0f) == common.GPS_TYPE_SERIAL ||
			(globalStatus.GPS_detected_type&0x0f) == common.GPS_TYPE_SOFTRF_DONGLE ||
			(globalStatus.GPS_detected_type&0x0f) == common.GPS_TYPE_SOFTRF_AT65) {
		// RVT: Verified
		fq := SituationData{}
		data, err := parseNMEALine_PGRMZ(x, &fq)

		if err != nil && (!isTempPressValid() || (mySituation.BaroSourceType != common.BARO_TYPE_BMP280 &&
			mySituation.BaroSourceType != common.BARO_TYPE_OGNTRACKER)) {
			mySituation.muBaro.Lock()
			mySituation.BaroPressureAltitude = data.BaroPressureAltitude // meters to feet
			mySituation.BaroLastMeasurementTime = data.BaroLastMeasurementTime
			mySituation.BaroSourceType = data.BaroSourceType
			mySituation.muBaro.Unlock()
		}

		return err == nil
	}

	// If we've gotten this far, the message isn't one that we can use.
	return false
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// GPS Helpers
//////////////////////////////////////////////////////////////////////////////////////////////////////

/*
isGPSValid returns true only if a valid position fix has been seen in the last 3 seconds,
and if the GPS subsystem has recently detected a GPS device.

If false, 'GPSFixQuality` is set to 0 ("No fix"), as is the number of satellites in solution.
*/

func resetGPSMySituation() {
	mySituation.GPSFixQuality = 0
	mySituation.GPSSatellites = 0
	mySituation.GPSSatellitesSeen = 0
	mySituation.GPSSatellitesTracked = 0
	mySituation.GPSHorizontalAccuracy = 999999
	mySituation.GPSVerticalAccuracy = 999999
	mySituation.GPSNACp = 0
	mySituation.GPSHDop = 0
	mySituation.GPSVDop = 0
	mySituation.GPSPositionSampleRate = 0;	
	mySituation.GPSTurnRate = 0;
	mySituation.GPSAltitudeMSL = 0;
}

/** Reset GOS global status
 */
func resetGPSGlobalStatus() {
	// Reset global status
	globalStatus.GPS_connected = false
	globalStatus.GPS_source = 0
	globalStatus.GPS_detected_type = 0
	globalStatus.GPS_NetworkRemoteIp = ""
	globalStatus.GPS_source_name = ""

	// reset my situation
	resetGPSMySituation()
	updateStatus()

	// Remove all satellites
	Satellites = make(map[string]SatelliteInfo)
}

func isGPSConnected() bool {
	return stratuxClock.Since(mySituation.GPSLastValidNMEAMessageTime) < GPS_CONSIDERING_CONNECTED
}

func isGPSValid() bool {
	return stratuxClock.Since(mySituation.GPSLastFixLocalTime) < GPS_VALID_TIME &&
		globalStatus.GPS_connected && mySituation.GPSFixQuality > 0
}

/*
isGPSGroundTrackValid returns true only if a valid ground track was obtained in the last 3 seconds,
and if NACp >= 9.
*/

func isGPSGroundTrackValid() bool {
	return isGPSValid() &&
		(mySituation.GPSHorizontalAccuracy < 30)
}

func isGPSClockValid() bool {
	return !mySituation.GPSLastGPSTimeStratuxTime.IsZero() && stratuxClock.Since(mySituation.GPSLastGPSTimeStratuxTime).Seconds() < 15
}

func isAHRSValid() bool {
	// If attitude information gets to be over 1 second old, declare invalid.
	// If no GPS then we won't use or send attitude information.
	return isGPSValid() && stratuxClock.Since(mySituation.AHRSLastAttitudeTime).Seconds() < 1
}

func isTempPressValid() bool {
	return isTempPressValid2(mySituation)
}

func isTempPressValid2(mySitu SituationData) bool {
	return stratuxClock.Since(mySitu.BaroLastMeasurementTime).Seconds() < 15
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// OGN
//////////////////////////////////////////////////////////////////////////////////////////////////////

/**
* request a 5Hz Navigation rate for OGN tracker and request it's configuration
 */
func (s *GPSDeviceManager) requestOgnTrackerConfiguration(name string) {

	// Request navrate of 5Hz
	s.configureGPS(gps.TXMessage{
		Message: []byte(common.AppendNmeaChecksum("$POGNS,NavRate=5") + "\r\n"),
		Name:    name,
	})

	// Request stored OGN tracker configuration to be send back to us
	s.configureGPS(gps.TXMessage{
		Message: []byte(s.getOgnTrackerConfigQueryString()),
		Name:    name,
	})
}

// Get OGN string to configure OGN connected device with our settings
func (s *GPSDeviceManager) getOgnTrackerConfigString(name string) string {
	msg := fmt.Sprintf("$POGNS,Address=0x%s,AddrType=%d,AcftType=%d,Pilot=%s,Reg=%s,TxPower=%d,Hard=STX,Soft=%s",
		globalSettings.OGNAddr, globalSettings.OGNAddrType, globalSettings.OGNAcftType, globalSettings.OGNPilot, globalSettings.OGNReg, globalSettings.OGNTxPower, stratuxVersion[1:])
	msg = common.AppendNmeaChecksum(msg)
	return msg + "\r\n"
}

// Request reading of the stored settings
func (s *GPSDeviceManager) getOgnTrackerConfigQueryString() string {
	return common.AppendNmeaChecksum("$POGNS") + "\r\n"
}

// Configure OGN connected device with our settings
func (s *GPSDeviceManager) configureOgnTrackerFromSettings(name string) {

	cfg := s.getOgnTrackerConfigString(name)
	log.Printf("GPS: Configuring OGN Tracker: %s ", cfg)

	s.configureGPS(gps.TXMessage{
		Message: []byte(cfg),
		Name:    name,
	})

	s.configureGPS(gps.TXMessage{
		Message: []byte(s.getOgnTrackerConfigQueryString()),
		Name:    name,
	})
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// GPSDeviceStatus
//////////////////////////////////////////////////////////////////////////////////////////////////////

/**
Returns true when any device has a valid fix
*/
func (d *GPSDeviceStatus) hasValidFix() bool {
	return d.gpsFixQuality > 0 &&
		stratuxClock.Since(d.gpsLastSeen) < GPS_TIME_SOURCE &&
		stratuxClock.Since(d.gpsLastGoodFix) < GPS_FIX_TIME
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// GPSDeviceManager
//////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Configure all GPS_TYPE_OGNTRACKER connected devices with our settings from globalSettings
 * Note: Currently most lickly it's only OGN on serial that can handle this
 */
func (s *GPSDeviceManager) ConfigureOgnTrackerFromSettings() {
	for entry := range s.discoveredDevices.IterBuffered() {
		v := entry.Val.(gps.DiscoveredDevice)
		if v.GpsDetectedType == common.GPS_TYPE_OGNTRACKER {
			s.configureOgnTrackerFromSettings(v.Name)
		}
	}
}

// Find a gps Source with a fix
// Not thread safe, only call from within a lock
// When GpsSource == 0 we get the first GPS with a fix
func (s *GPSDeviceManager) gpsDeviceWithFix(GpsSource uint16) (string, GPSDeviceStatus) {
	// TODO: RVT: We should sort on a GPS devices with the best fix ??
	for k, v := range s.gpsDeviceStatus {
		if (v.gpsSource == GpsSource || GpsSource == 0) && v.hasValidFix() {
			return k, v
		}
	}
	return "", GPSDeviceStatus{}
}

// Find any GPS with a fix with preference to gpsSource
// Not thread safe, only call from within a lock
func (s *GPSDeviceManager) anyGpsDeviceWithFix(gpsSource uint16) (k string, v GPSDeviceStatus) {
	k, v = s.gpsDeviceWithFix(gpsSource)
	if !v.hasValidFix() {
		k, v = s.gpsDeviceWithFix(0)
	}
	return
}

/**
Maintain and decide on what gps source to use
When we do not have any GPS, we just pick one from the list that has a fix, if not we just pick the first if any
**/
func (s *GPSDeviceManager) maintainPreferredGPSDevice() {
	maintenanceJob := func() {
		s.m.Lock()
		defer s.m.Unlock()

		currentGPSSource, hasDeviceConfig := s.gpsDeviceStatus[s.currentGPSName]

		// Handle situation where we do not have any current GPS
		if s.currentGPSName == "" || !hasDeviceConfig {
			gpsWithFix, _ := s.anyGpsDeviceWithFix(uint16(globalSettings.GPSPreferredSource))
			// Pick the first GPS because non have a fix
			if gpsWithFix == "" {
				for arbitaryItem := range s.gpsDeviceStatus {
					s.currentGPSName = arbitaryItem
					break
				}
			} else {
				s.currentGPSName = gpsWithFix
			}
		}

		// Find the current GPS device and validate if it is still valid
		if hasDeviceConfig {
			// Verify of we still have a good fix from the current GPS device, if not we find a other
			// GPS with a good fix, if not found then reset GPS system
			GPSPreferredSource := uint16(globalSettings.GPSPreferredSource)
			if !currentGPSSource.hasValidFix() {
				anyGpsName, _ := s.anyGpsDeviceWithFix(GPSPreferredSource)
				if anyGpsName != "" {
					s.currentGPSName = anyGpsName
				}
			}

			// Verify if the current GPS source might not be preferred, if so then lookup a GPS
			// source that is preferrred and has a fix, if found use the preferred source
			if currentGPSSource.gpsSource != GPSPreferredSource {
				possiblePreferredSource, _ := s.gpsDeviceWithFix(GPSPreferredSource)
				if possiblePreferredSource != "" {
					s.currentGPSName = possiblePreferredSource
				}
			}
		}

		// Loop over all sources and see if we have any GPS still connected, if not reset teh GPS
		anyGPSWIthActivity := false
		for gpsDeviceName := range s.gpsDeviceStatus {
			if stratuxClock.Since(s.gpsDeviceStatus[gpsDeviceName].gpsLastSeen) < GPS_CONSIDERING_CONNECTED {
				anyGPSWIthActivity = true
			} else {
				// When this is the current device, it's expired since it was a while since we last seen it then make it not current anymore
				if gpsDeviceName == s.currentGPSName {
					s.currentGPSName = ""
				}
			}
		}
		if !anyGPSWIthActivity {
			resetGPSGlobalStatus()
		}
	}

	// Run maintenance tasks every 1 second
	timer := time.NewTicker(1000 * time.Millisecond)
	for {
		select {
		case <-timer.C:
			maintenanceJob()
		}
	}
}

/**
Send a message to an attached GPS
*/
func (s *GPSDeviceManager) configureGPS(txMessage gps.TXMessage) {
	s.txMessageCh <- txMessage
}

/**
Listen to rxMessageCh and process incomming NMEA message from GPS sources
*/
func (s *GPSDeviceManager) rxMessageHandler() {
	log.Printf("GPS: rxMessageHandler: Started")
	for {
		rxMessage := <-s.rxMessageCh

		s.m.Lock()

		// Load what we currently have seen about this GPS, create a new record if it was never seen
		thisGPS, hasDeviceConfig := s.gpsDeviceStatus[rxMessage.Name]

		if !hasDeviceConfig {
			thisGPS = GPSDeviceStatus{
				gpsFixQuality:  0,
				gpsLastGoodFix: time.Time{},
				gpsCRCErrors:   0,
			}
		}
		thisGPS.gpsLastSeen = stratuxClock.Time

		// Validate NMEA sentence, and ignore if the CRC was wrong
		l_valid, validNMEAcs := common.ValidateNMEAChecksum(rxMessage.NmeaLine)
		if !validNMEAcs {
			if len(l_valid) > 0 {
				log.Printf("GPS error. Invalid NMEA string: %s %s\n", l_valid, rxMessage.NmeaLine) // remove log message once validation complete
			}
			thisGPS.gpsCRCErrors++
		} else {
			nmeaSlice := strings.Split(l_valid, ",")

			// Test if the Name matches that of a discvered GPS device, only these are allowed to get processed
			if deviceRaw, ok := s.discoveredDevices.Get(rxMessage.Name); ok {
				// Always process OGN
				ognPublishNmea(rxMessage.NmeaLine)

				discoveredDevice := deviceRaw.(gps.DiscoveredDevice)
				if common.StringInSlice(nmeaSlice[0], ALWAYS_PROCESS_NMEAS) {
					// Some commands that do not affect GPS location services can always and should bebe processed
					processFlarmNmeaMessage(nmeaSlice)
					if ok := s.processNMEALine(l_valid, rxMessage.Name, discoveredDevice); ok {
						registerSituationUpdate()
					}
				} else {

					// Process all NMEA for the current GPS
					if s.currentGPSName == rxMessage.Name {
						globalStatus.GPS_detected_type = discoveredDevice.GpsDetectedType | common.GPS_PROTOCOL_NMEA
						if ok := s.processNMEALine(l_valid, rxMessage.Name, discoveredDevice); ok {
							registerSituationUpdate()
							globalStatus.GPS_source_name = rxMessage.Name
							globalStatus.GPS_connected = true
							globalStatus.GPS_source = uint(discoveredDevice.GpsSource)
							// We should add the GPS to gpsDeviceStatus directly when we discover it!
							thisGPS.gpsSource = discoveredDevice.GpsSource // TODO: RVT: We need to gpsSource in the gpsDevice status for sorting, we could also just use the map?
						}
					}

					// Check and remmeber this GPS fix quality
					var situ SituationData;
					situ.GPSFixQuality = 255 // We would not expect GPSFixQuality of 255, so we use it as a marker to see if it was changed
					situ, _ = parseNMEALine_GNGGA_GPGGA(nmeaSlice, &situ)
					situ, _ = parseNMEALine_GNGSA_GPGSA_GLGSA_GAGSA_GBGSA(nmeaSlice, &situ)
					if situ.GPSFixQuality != 255 {
						thisGPS.gpsFixQuality = situ.GPSFixQuality
						if thisGPS.gpsFixQuality > 0 {
							thisGPS.gpsLastGoodFix = stratuxClock.Time
						}
					}
				}
			} else {
				log.Printf("Warning: Receive GPS before discovery for %s", rxMessage.Name) // remove log message once validation complete
			}
		}

		s.gpsDeviceStatus[rxMessage.Name] = thisGPS
		s.m.Unlock()
	}
}

/**
Watch the global current settings for a few variables and request a re-init of all GPS systems
when any of the globalSettings that are monitored are modified
*/
func (s *GPSDeviceManager) globalConfigChangeWatcher() {
	log.Printf("GPS: globalConfigChangeWatcher: Started")

	checkAndReInit := func() {
		x := s.settingsCopy.BleGPSEnabled != globalSettings.BleGPSEnabled
		y := s.settingsCopy.GPSPreferredSource != globalSettings.GPSPreferredSource
		z := s.settingsCopy.BleEnabledDevices != globalSettings.BleEnabledDevices
		d := s.settingsCopy.DEBUG != globalSettings.DEBUG
		g := s.settingsCopy.GPS_Enabled != globalSettings.GPS_Enabled

		reInit := x || y || z || d || g

		if reInit {

			s.qh.Quit()
			s.settingsCopy = globalSettings

			// At this point new threads can start all adapters, but we assume here
			// that we will be faster resetting the GPS device
			s.gpsDeviceStatus = make(map[string]GPSDeviceStatus)
			resetGPSGlobalStatus()
		}
	}

	timer := time.NewTicker(2000 * time.Millisecond)
	for {
		<-timer.C
		checkAndReInit()
	}
}

/**
Configure and enable GPS input sources
*/
func (s *GPSDeviceManager) configureGPSSubsystems() {
	log.Printf("GPS: configureGPSSubsystems: Started")
	s.qh.Add()
	defer s.qh.Done()

	bleGPSDevice := gps.NewBleGPSDevice(s.rxMessageCh, s.discoveredDevicesCh)
	serialGPSDevice := gps.NewSerialGPSDevice(s.rxMessageCh, s.discoveredDevicesCh, globalSettings.DEBUG)
	networkGPSDevice := gps.NewNetworkGPSDevice(s.rxMessageCh, s.discoveredDevicesCh)

	defer func() {
		log.Printf("GPS: configureGPSSubsystems stopping")
		bleGPSDevice.Stop()
		serialGPSDevice.Stop()
		networkGPSDevice.Stop()
		log.Printf("GPS: configureGPSSubsystems Stopped")
	}()

	if globalSettings.BleGPSEnabled {
		log.Printf("GPS: configureGPSSubsystems: Enable Bluetooth devices")
		go bleGPSDevice.Run(strings.Split(globalSettings.BleEnabledDevices, ","))
	}

	if globalSettings.GPS_Enabled {
		log.Printf("GPS: configureGPSSubsystems: Enable USB/Serial devices")
		go serialGPSDevice.Run()
	}

	log.Printf("GPS: configureGPSSubsystems: Enable Network devices")
	go networkGPSDevice.Run()

	<-s.qh.C
}

/**
Maintain a list of configured and found GPS devices and set the list in globalStatus
*/
func (s *GPSDeviceManager) handleDeviceDiscovery() {
	handleNewDiscoveredDevice := func(discoveredDevice gps.DiscoveredDevice) {

		// Copy TXChannel if one previously was already known because device discovery might not always have a TXCHannel added
		if previousRaw, ok := s.discoveredDevices.Get(discoveredDevice.Name); ok {
			previous := previousRaw.(gps.DiscoveredDevice)
			if previous.HasTXChannel && !discoveredDevice.HasTXChannel {
				discoveredDevice.HasTXChannel = true
				discoveredDevice.TXChannel = previous.TXChannel
			}
			// If the last Version was a upgraded version, then copy the device type
			// This is to avoid any device discovery send of versions that where not upgraded
			if previous.IsTypeUpgrade {
				discoveredDevice.IsTypeUpgrade = true
				discoveredDevice.GpsDetectedType = previous.GpsDetectedType
			}

		}
		discoveredDevice.LastDiscoveryMessage = stratuxClock.Milliseconds
		s.discoveredDevices.Set(discoveredDevice.Name, discoveredDevice)

		// Build list of GPS_Discovery source for the UI
		deviceList := []gps.DiscoveredDeviceDTO{}
		for entry := range s.discoveredDevices.IterBuffered() {
			v := entry.Val.(gps.DiscoveredDevice)
			// Show all devices we have seen since uptime
			deviceList = append(deviceList, gps.DiscoveredDeviceDTO{
				Name:                 v.Name,
				Connected:            v.Connected,
				LastDiscoveryMessage: v.LastDiscoveryMessage,
				GpsDetectedType:      v.GpsDetectedType,
				GpsSource:            v.GpsSource,
			})
		}
		globalStatus.GPS_Discovery = deviceList
	}

	sendGPSAMessage := func(message *gps.TXMessage) {
		if deviceRaw, ok := s.discoveredDevices.Get(message.Name); ok {
			device := deviceRaw.(gps.DiscoveredDevice)
			if device.Connected && device.HasTXChannel {
				device.TXChannel <- []byte(message.Message)
			}
		}
	}

	for {
		select {
		case discoveredDevice := <-s.discoveredDevicesCh:
			handleNewDiscoveredDevice(discoveredDevice)
			break
		case txConfig := <-s.txMessageCh:
			sendGPSAMessage(&txConfig)
			break
		}
	}
}

func (s *GPSDeviceManager) Run() {
	Satellites = make(map[string]SatelliteInfo)
	log.Printf("GPS: Listen: Started")
	// message := make(chan DiscoveredDevice, 20)

	go s.systemTimeSetter.Calibrate()
	go s.systemTimeSetter.Run()
	go s.globalConfigChangeWatcher()
	go s.rxMessageHandler()
	go s.maintainPreferredGPSDevice()
	go s.handleDeviceDiscovery()
	for {
		s.configureGPSSubsystems()
		time.Sleep(1000 * time.Millisecond)
	}
}
