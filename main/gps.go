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
	"reflect"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/b3nn0/stratux/v2/common"
	"github.com/b3nn0/stratux/v2/gps"
	cmap "github.com/orcaman/concurrent-map/v2"
	"golang.org/x/exp/slices"
)

// Array of NMEA lines we would always want to process from any attached GPS device
// This should never contain any NMEA codes for GPS Location data because we want location data to be comming from one GPS only
func alwaysProcessTheseNMEAs() []string {
	return []string{"PFLAU", "PFLAA", "POGNS", "POGNR", "POGNB", "PSOFT"}
}

const (
	GPS_FIX_TIME              = 5000 * time.Millisecond // Time we expect a GPS satelite to have a valid fix
	GPS_TIME_SOURCE           = 2000 * time.Millisecond // Time we expect the GPS to be a valid source before we reconsider other GPS location sources
	GPS_VALID_TIME            = 3000 * time.Millisecond // TIme we consider a GPS data source to be valid
	GPS_CONSIDERING_CONNECTED = 8000 * time.Millisecond // Time we expect a GPS as beeing connected to stratux
	FIX_QUALITY_NOFIX         = 0                       // No Fix
	FIX_QUALITY_3DGPS         = 1                       // 3DGPS
	FIX_QUALITY_AGPS          = 2                       // SBAS/WAAS
)

const (
	SAT_TYPE_UNKNOWN = 0  // default type
	SAT_TYPE_GPS     = 1  // GPxxx; NMEA IDs 1-32
	SAT_TYPE_GLONASS = 2  // GLxxx; NMEA IDs 65-96
	SAT_TYPE_GALILEO = 3  // GAxxx; NMEA IDs
	SAT_TYPE_BEIDOU  = 4  // GBxxx; NMEA IDs 201-235
	SAT_TYPE_QZSS    = 5  // QZSS
	SAT_TYPE_SBAS    = 10 // NMEA IDs 33-54
)

const (
	BARO_TYPE_NONE         = 0 // No baro present
	BARO_TYPE_BMP280       = 1 // Stratux AHRS module or similar internal baro
	BARO_TYPE_OGNTRACKER   = 2 // OGN Tracker with baro pressure
	BARO_TYPE_NMEA         = 3 // Other NMEA provider that reports $PGRMZ (SoftRF)
	BARO_TYPE_ADSBESTIMATE = 4 // If we have no baro, we will try to estimate baro pressure from ADS-B targets reporting GnssDiffFromBaroAlt (HAE<->Baro difference)
)

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

type GPSDeviceStatus struct {
	name           string // Mandatory: Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	gpsSource      uint16
	gpsFixQuality  uint8
	gpsLastSeen    time.Time
	gpsLastGoodFix time.Time
	gpsCRCErrors   uint64
}

func (d *GPSDeviceStatus) hasValidFix() bool {
	return d.gpsFixQuality > 0 &&
		stratuxClock.Since(d.gpsLastSeen) < GPS_TIME_SOURCE &&
		stratuxClock.Since(d.gpsLastGoodFix) < GPS_FIX_TIME // Would it not be already good enough to just check the fix?
}

type gpsDevice interface {
	Scan(leh *common.ExitHelper)
    Stop() 
}

type GPSDeviceManager struct {
	gpsDeviceStatus  cmap.ConcurrentMap[string, GPSDeviceStatus] // Map of device status when they are sending us message
	settingsCopy     settings                            // Copy of the global Settings to help decide if some varas are changed
	currentGPSNameCh chan string                         // name of the current GPS device we receive location data from
	eh               *common.ExitHelper                  // Helper to quick various routines during reconfiguratios

	rxMessageCh chan gps.RXMessage // Channel used by all GPS devices to receive NMEA data from

	systemTimeSetter  *gps.OSTimeSetter
	discoveredDevices cmap.ConcurrentMap[string, gps.DiscoveredDevice]

	ognTrackerConfigured bool

	deviceList []gpsDevice
	scanMutex			 *sync.Mutex
}

var gpsPerf gpsPerfStats
var myGPSPerfStats []gpsPerfStats
var Satellites map[string]SatelliteInfo

func NewGPSDeviceManager() GPSDeviceManager {

	return GPSDeviceManager{
		gpsDeviceStatus:  cmap.New[GPSDeviceStatus](),
		settingsCopy:     globalSettings,
		currentGPSNameCh: make(chan string),
		eh:               common.NewExitHelper(),

		rxMessageCh: make(chan gps.RXMessage, 20),

		systemTimeSetter:  gps.GetOSTimeSetter(),
		discoveredDevices: cmap.New[gps.DiscoveredDevice](),

		ognTrackerConfigured: false,

		deviceList: make([]gpsDevice, 0),

		scanMutex: &sync.Mutex{},
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
			if thisSatellite.Type == SAT_TYPE_SBAS {
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
	// 30 seconds @ 10 Hz for UBX, 30 seconds @ 5 Hz for MTK or SIRF with 2x messages per 200 ms)
	// This might not be true for all GPS systems, is this important?
	if lenGPSPerfStats > 299 {
		myGPSPerfStats = myGPSPerfStats[(lenGPSPerfStats - 299):] // remove the first n entries if more than 300 in the slice
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Process a NMEA line and update strauc
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Process a NMEA line and update stratux internal variables
func (s *GPSDeviceManager) processNMEALine(l string, deviceDiscovery gps.DiscoveredDevice) (sentenceUsed bool) {
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
		data, err := parseNMEALine_GNVTG_GPVTG(x, mySituation)
		if err == nil {
			mySituation = data
		}
		return err == nil
		// ############################################# GNGGA GPGGA #############################################
	} else if (x[0] == "GNGGA") || (x[0] == "GPGGA") { // Position fix.
		data, err := parseNMEALine_GNGGA_GPGGA(x, mySituation)
		if err == nil {
			mySituation = data

			//
			t := mySituation.GPSLastFixSinceMidnightUTC
			hh := int(t / 3600)
			mm := int(t-float32(hh*3600)) / 60
			ss := int(t - float32(hh*3600) - float32(mm*60))
			sss := t - float32(hh*3600) - float32(mm*60) - float32(ss)
			tWithoutOffset := time.Date(
				mySituation.GPSTime.Year(),
				mySituation.GPSTime.Month(),
				mySituation.GPSTime.Day(),
				hh,
				mm,
				ss,
				int(sss*1e9),
				mySituation.GPSTime.Location())

			gpsTime := tWithoutOffset.Add(deviceDiscovery.GPSTimeOffsetPPS)
			mySituation.GPSTime = gpsTime
			thisGpsPerf.nmeaTime = mySituation.GPSLastFixSinceMidnightUTC
			thisGpsPerf.alt = float32(mySituation.GPSAltitudeMSL)
			thisGpsPerf.msgType = x[0]
			updateGPSPerfmStat(thisGpsPerf)

			// Always set time from GXGGA
			s.systemTimeSetter.SetTime(tWithoutOffset, deviceDiscovery.GPSTimeOffsetPPS, deviceDiscovery.Name)
			stratuxClock.SetRealTimeReference(gpsTime)
			// Inform we received time for GXGGA and we can use that as a time source
			if !deviceDiscovery.SetTimeFromGGAOnly {
				gps.GetServiceDiscovery().TimeFromGXGGAOnly(deviceDiscovery.Name, true)
			}

			if globalSettings.DEBUG {
				// To test for time settings and what times we receive over GPS
				log.Printf("GPS: %s %s %02d:%02d:%02d:%02d\n", deviceDiscovery.Name, l, hh, mm, ss, int(sss*1000))
			}

		}
		return err == nil
		// ############################################# GNRMC GPRMC #############################################
	} else if (x[0] == "GNRMC") || (x[0] == "GPRMC") { // Recommended Minimum data.
		data, err := parseNMEALine_GNRMC_GPRMC(x, mySituation)
		if err == nil && data.GPSFixQuality != FIX_QUALITY_NOFIX {
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
			tWithoutOffset := mySituation.GPSTime
			mySituation.GPSTime = tWithoutOffset.Add(deviceDiscovery.GPSTimeOffsetPPS)
			// Only set time if we did not receive any timing from GXGGA messages.
			// We do this because some GPS devices like AT65 only send GXGGA on the seconds, like PPS
			if len(x[9]) == 6 && !deviceDiscovery.SetTimeFromGGAOnly {
				s.systemTimeSetter.SetTime(tWithoutOffset, deviceDiscovery.GPSTimeOffsetPPS, deviceDiscovery.Name)
				stratuxClock.SetRealTimeReference(mySituation.GPSTime)
				if globalSettings.DEBUG {
					// To test for time settings and what times we receive over GPS
					t := mySituation.GPSLastFixSinceMidnightUTC
					hh := int(t / 3600)
					mm := int(t-float32(hh*3600)) / 60
					ss := int(t - float32(hh*3600) - float32(mm*60))
					sss := t - float32(hh*3600) - float32(mm*60) - float32(ss)
					log.Printf("GPS: %s %s %02d:%02d:%02d:%02d\n", deviceDiscovery.Name, l, hh, mm, ss, int(sss*1000))
				}
			}

			updateGPSPerfmStat(thisGpsPerf)
			setDataLogTimeWithGPS(mySituation)
		}
		return err == nil
		// ############################################# GNGSA GPGSA GLGSA GAGSA GBGSA #############################################
	} else if (x[0] == "GNGSA") || (x[0] == "GPGSA") || (x[0] == "GLGSA") || (x[0] == "GAGSA") || (x[0] == "GBGSA") { // Satellite data.
		data, err := parseNMEALine_GNGSA_GPGSA_GLGSA_GAGSA_GBGSA(x, mySituation)
		if err == nil && data.GPSFixQuality != FIX_QUALITY_NOFIX {
			if data.GPSFixQuality == FIX_QUALITY_AGPS { // Rough 95% confidence estimate for SBAS solution
				if deviceDiscovery.GPSDetectedType == gps.GPS_TYPE_UBX9 {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 3.0) // ublox 9
				} else {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 4.0) // ublox 6/7/8
				}
			} else { // Rough 95% confidence estimate non-SBAS solution
				if deviceDiscovery.GPSDetectedType == gps.GPS_TYPE_UBX9 {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 4.0) // ublox 9
				} else {
					data.GPSHorizontalAccuracy = float32(data.GPSHDop * 5.0) // ublox 6/7/8
				}
			}

			data.GPSVerticalAccuracy = data.GPSVDop * 5.0 // Rough 95% confidence estimate
			data.GPSNACp = calculateNACp(data.GPSHorizontalAccuracy)

			updateSatellites(x)
			data.GPSSatellites, data.GPSSatellitesTracked, data.GPSSatellitesSeen = updateConstellation()

			mySituation = data

		}
		return err == nil
		// ############################################# GPGSV GLGSV GAGSV GBGSV #############################################
	} else if (x[0] == "GPGSV") || (x[0] == "GLGSV") || (x[0] == "GAGSV") || (x[0] == "GBGSV") { // GPS + SBAS or GLONASS or Galileo or Beidou satellites in view message.
		data, err := parseNMEALine_GPGSV_GLGSV_GAGSV_GBGSV(x, mySituation)
		if err == nil {

			updateSatellitesInView(x)
			data.GPSSatellites, data.GPSSatellitesTracked, data.GPSSatellitesSeen = updateConstellation()

			mySituation = data

		}
		return err == nil

		// ############################################# POGNB #############################################
	} else if x[0] == "POGNB" {
		data, err := parseNMEALine_POGNB(x, mySituation)
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
		// When PSOFT is send, we know the device is a SOFTRF
		data, err := parseNMEALine_PSOFT(x, mySituation)
		if err == nil {
			mySituation = data

			// Detect GPS type
			if x[1] == "AT65" && deviceDiscovery.GPSSource == gps.GPS_SOURCE_BLUETOOTH {
				gps.GetServiceDiscovery().Send(gps.DiscoveredDevice{
					Name:             deviceDiscovery.Name,
					Content:          gps.CONTENT_OFFSET_PPS | gps.CONTENT_TYPE,
					GPSDetectedType:  gps.GPS_TYPE_SOFTRF_AT65,
					GPSTimeOffsetPPS: 250 * time.Millisecond,
				})
			} else {
				gps.GetServiceDiscovery().Send(gps.DiscoveredDevice{
					Name:             deviceDiscovery.Name,
					Content:          gps.CONTENT_TYPE,
					GPSDetectedType:  gps.GPS_TYPE_SOFTRF_DONGLE,
				})
			}

			// Request config
			s.configureGPS(common.MakeNMEACmd(fmt.Sprintf("PSRFC,?")), deviceDiscovery.Name)
			
		}
		return err == nil
		} else if x[0] == "PSRFC" {
		// ############################################# PSRFC #############################################} else if x[0] == "PSOFT" {
		// When PSRFC we nee dto validate it's configuration and if needed modify for stratux usage
		data, err := parseNMEALine_PSRFC(x, mySituation)
		if err == nil {
			mySituation = data

			// Detect HW configuration and modify Address if different from Stratux
			requiredAddr, _ := strconv.ParseUint(globalSettings.OGNAddr, 16, 32)

			protocol := 0 // 0 Legacy(FLARM), 1 OGN, 2 P3I, 3 FANET, 4 UAT
			sentence := fmt.Sprintf("PSRFC,%s,%s,%d,%s,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%06X",
				x[1],  // Version
				x[2],  // Mode
				protocol, 
				x[4],  // Region
				globalSettings.OGNAcftType, // Aircraft type
				x[6],  // Alarm
				x[7],  // txPower
				x[8],  // Volume
				x[9],  // Pointer
				x[10], // NMEA GNSS
				x[11], // Private
				x[12], // NMEA Legacy
				x[13], // NMEA Sensors
				x[14], // gps source 5 BT 4 USB // NMEA Output
				x[15], // GDL90 Out
				x[16], // D1090 Out
				x[17], // Stealth
				x[18], // noTrack
				x[19], // Power Save
				requiredAddr)

			if (sentence != l) {
				log.Printf("WARNING: SoftRF incorrect for stratux, reconfiguring with %s", sentence)
				s.configureGPS(common.MakeNMEACmd(sentence), deviceDiscovery.Name)
			}

		}
		return err == nil
		// ############################################# POGNR #############################################		
	} else if x[0] == "POGNR" {
		// Only sent by OGN tracker. We use this to detect that OGN tracker is connected and configure it as needed
		if !s.ognTrackerConfigured {
			s.ognTrackerConfigured = true
			s.requestOgnTrackerConfiguration(deviceDiscovery.Name)
		}

		return true
		// ############################################# POGNS #############################################
	} else if x[0] == "POGNS" {
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
		((globalStatus.GPS_detected_type&0x0f) == gps.GPS_TYPE_SERIAL ||
			(globalStatus.GPS_detected_type&0x0f) == gps.GPS_TYPE_SOFTRF_DONGLE ||
			(globalStatus.GPS_detected_type&0x0f) == gps.GPS_TYPE_SOFTRF_AT65) {
		fq := SituationData{}
		data, err := parseNMEALine_PGRMZ(x, fq)

		if err == nil && (!isTempPressValid() ||
			(mySituation.BaroSourceType != BARO_TYPE_BMP280 &&
				mySituation.BaroSourceType != BARO_TYPE_OGNTRACKER)) {
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

func resetGPSMySituation() {
	mySituation.GPSFixQuality = FIX_QUALITY_NOFIX
	mySituation.GPSSatellites = 0
	mySituation.GPSSatellitesSeen = 0
	mySituation.GPSSatellitesTracked = 0
	mySituation.GPSHorizontalAccuracy = 999999
	mySituation.GPSVerticalAccuracy = 999999
	mySituation.GPSNACp = 0
	mySituation.GPSHDop = 0
	mySituation.GPSVDop = 0
	mySituation.GPSPositionSampleRate = 0
	mySituation.GPSTurnRate = 0
	mySituation.GPSAltitudeMSL = 0
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

/*
isGPSValid returns true only if a valid position fix has been seen in the last GPS_VALID_TIME,
and if the GPS subsystem has recently detected a GPS device.
*/

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
	s.configureGPS([]byte(common.AppendNmeaChecksum("$POGNS,NavRate=5") + "\r\n"), name)

	// Request stored OGN tracker configuration to be send back to us
	s.configureGPS([]byte(s.getOgnTrackerConfigQueryString()), name)
}

// Get OGN string to configure OGN connected device with our settings
func getOgnTrackerConfigString() string {
	msg := fmt.Sprintf("$POGNS,Address=0x%s,AddrType=%d,AcftType=%d,Pilot=%s,Reg=%s,TxPower=%d,Hard=STX,Soft=%s",
		globalSettings.OGNAddr, globalSettings.OGNAddrType, globalSettings.OGNAcftType, globalSettings.OGNPilot, globalSettings.OGNReg, globalSettings.OGNTxPower, stratuxVersion[1:])
	msg = common.AppendNmeaChecksum(msg)
	return msg + "\r\n"
}

// Request reading of the stored settings in OGN
func (s *GPSDeviceManager) getOgnTrackerConfigQueryString() string {
	return common.AppendNmeaChecksum("$POGNS") + "\r\n"
}

// Configure OGN connected device with our settings
func (s *GPSDeviceManager) configureOgnTrackerFromSettings(name string) {
	cfg := getOgnTrackerConfigString()
	log.Printf("GPS: Configuring OGN Tracker: %s ", cfg)

	s.configureGPS([]byte(cfg), name)

	s.configureGPS([]byte(s.getOgnTrackerConfigQueryString()), name)
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
		if entry.Val.GPSDetectedType == gps.GPS_TYPE_OGNTRACKER {
			s.configureOgnTrackerFromSettings(entry.Key)
		}
	}
}

// Return the gpsDeviceStatus as List and sorted by importance
func (s *GPSDeviceManager) gpsDeviceStatusAsList(GPSSource uint16) []GPSDeviceStatus {
	statusList := make([]GPSDeviceStatus, 0)
	for entry := range s.gpsDeviceStatus.IterBuffered() {
		statusList = append(statusList, entry.Val)
	}

	sort.Slice(statusList, func(i, j int) bool {
		di := statusList[i]
		return di.hasValidFix()
	})

	sort.Slice(statusList, func(i, j int) bool {
		di := statusList[i]
		dj := statusList[j]
		return di.gpsFixQuality > dj.gpsFixQuality
	})

	sort.Slice(statusList, func(i, j int) bool {
		di := statusList[i]
		return di.gpsSource == GPSSource
	})

	sort.Slice(statusList, func(i, j int) bool {
		di := statusList[i]
		dj := statusList[j]
		return di.gpsCRCErrors < dj.gpsCRCErrors
	})

	return statusList
}

// Find a gps Source with a fix, we prefer a source given by GPSSource
func (s *GPSDeviceManager) gpsDeviceWithFix(GPSSource uint16) (GPSDeviceStatus, bool) {

	for _, v := range s.gpsDeviceStatusAsList(GPSSource) {
		if (v.gpsSource == GPSSource || GPSSource == 0) && v.hasValidFix() {
			return v, true
		}
	}
	return GPSDeviceStatus{}, false
}

// Find any GPS with a fix, we prefer a source given by GPSSource
func (s *GPSDeviceManager) anyGpsDeviceWithFix(gpsSource uint16) (v GPSDeviceStatus, ok bool) {
	v, ok = s.gpsDeviceWithFix(gpsSource)
	if !ok {
		v, ok = s.gpsDeviceWithFix(0)
	}
	return
}

// Find any GPS devices that we receive data on, we prefer the GPS from gpsSource
func (s *GPSDeviceManager) anyGpsDevice(gpsSource uint16) (v GPSDeviceStatus, ok bool) {
	deviceList := s.gpsDeviceStatusAsList(gpsSource)
	if len(deviceList) > 0 {
		return deviceList[0], true
	}
	return GPSDeviceStatus{}, false
}

/**
Send a message to an attached GPS
*/
func (s *GPSDeviceManager) configureGPS(message []byte , name string) {
	if entry, ok := s.discoveredDevices.Get(name); ok {
		if entry.Connected && entry.CanTX() {
			entry.TXChannel <- message
		} else if globalSettings.DEBUG {
			log.Printf("Device %s does not have an TX Channel", name)
		}
	}
}

/**
Maintain and decide on what gps source to use
**/
func (s *GPSDeviceManager) maintainPreferredGPSDevice() {
	gpsName := ""
	// Decide what GPS device to use
	decidePreferredGPSDevice := func() {
		currentGPSName := gpsName
		GPSPreferredSource := uint16(globalSettings.GPSPreferredSource)

		// Get the current GPSDevice, if any..
		currentGPSDevice, ok := s.gpsDeviceStatus.Get(gpsName)
		reason := "No GPS device"
		if !ok {
			reason = "No device"
			gpsName = ""
		}

		// Test if the current Device still has a good fix, if not we find a new one
		// If we do not find any GPS device with a fix, we find a connected device
		if !currentGPSDevice.hasValidFix() {
			if otherGPSDevice, ok := s.anyGpsDeviceWithFix(GPSPreferredSource); ok {
				gpsName = otherGPSDevice.name
				reason = "Found a device with fix"
			} else if currentGPSName == "" {
				if anyDevice, ok := s.anyGpsDevice(GPSPreferredSource); ok {
					reason = "Selected best from connected devices"
					gpsName = anyDevice.name
				}
			}
		}

		// Verify if the current GPS source might not be preferred, if so then lookup a GPS
		// source that is preferrred and has a fix, if found use the preferred source
		if currentGPSDevice.gpsSource != GPSPreferredSource {
			if preferredDevice, ok := s.gpsDeviceWithFix(GPSPreferredSource); ok {
				gpsName = preferredDevice.name
				reason = "Selected best device from preferred source"
			}
		}

		// When we have a new GPS device, we configure it
		if currentGPSName != gpsName {
			whatGPS := currentGPSName
			if currentGPSName == "" {
				whatGPS = "no Device"
			}
			log.Printf("GPS device changed from [%s] to [%s]: %s", whatGPS, gpsName, reason)
			resetGPSGlobalStatus()
			s.currentGPSNameCh <- gpsName
		}
	}

	// Remove any outdated GPS devices from our list
	removeOutdatedGPSDevices := func() {
		// Loop over all sources and see if we have any GPS still connected, if not reset the GPS
		anyGPSWIthActivity := false
		for entry := range s.gpsDeviceStatus.IterBuffered() {
			if stratuxClock.Since(entry.Val.gpsLastSeen) < GPS_CONSIDERING_CONNECTED {
				anyGPSWIthActivity = true
			} else {
				// Remove from device if we did not see it for a while
				s.gpsDeviceStatus.Pop(entry.Key)
				// Just in case if a device forget's to send a device as disconnected via discovery we send a disconnect
				gps.GetServiceDiscovery().Connected(entry.Key, false)
			}
		}
		if !anyGPSWIthActivity {
			resetGPSGlobalStatus()
		}
	}

	// Run maintenance tasks every 1 second
	decideTimer := time.NewTicker(1000 * time.Millisecond)
	removeTimer := time.NewTicker(5000 * time.Millisecond)
	for {
		select {
		case <-decideTimer.C:
			decidePreferredGPSDevice()
		case <-removeTimer.C:
			removeOutdatedGPSDevices()
		}
	}
}

/**
Maintain a list of configured and found GPS devices and set the list in globalStatus
Also start a go routine that will send messages to GPS devices
*/
func (s *GPSDeviceManager) handleDeviceDiscovery() {
	for {
		discoveredDevice := <-gps.GetServiceDiscovery().C

		if previous, ok := s.discoveredDevices.Get(discoveredDevice.Name); ok {
			discoveredDevice = gps.GetServiceDiscovery().Merge(previous, discoveredDevice)
		}
		s.discoveredDevices.Set(discoveredDevice.Name, discoveredDevice)

		// Build list of GPS_Discovery source for the UI
		deviceList := []gps.DiscoveredDeviceDTO{}
		for entry := range s.discoveredDevices.IterBuffered() {
			v := entry.Val
			// Show all devices we have seen since uptime
			deviceList = append(deviceList, gps.DiscoveredDeviceDTO{
				Name:            v.Name,
				Connected:       v.Connected,
				GPSDetectedType: v.GPSDetectedType,
				GPSSource:       v.GPSSource,
				MAC:             v.MAC,
			})
		}
		globalStatus.GPS_Discovery = deviceList
	}
}

/**
Listen to rxMessageCh and process incomming NMEA message from GPS sources
*/
func (s *GPSDeviceManager) rxMessageHandler() {
	log.Printf("GPS: rxMessageHandler: Started")
	currentGPSName := ""

	processMessage := func(rxMessage gps.RXMessage) {
		// Get and a device status if it did not exist before
		thisGPS, hasDeviceConfig := s.gpsDeviceStatus.Get(rxMessage.Name)
		if !hasDeviceConfig {
			thisGPS = GPSDeviceStatus{
				name: rxMessage.Name,
			}
		}

		defer func() {
			thisGPS.gpsLastSeen = stratuxClock.Time
			s.gpsDeviceStatus.Set(rxMessage.Name, thisGPS)
		}()

		// Test if this device was discovered in the first place, if not ignore the whole message
		var discoveredDevice gps.DiscoveredDevice
		if dd, ok := s.discoveredDevices.Get(rxMessage.Name); ok {
			thisGPS.gpsSource = dd.GPSSource
			discoveredDevice = dd
		} else {
			log.Printf("Warning: Receive GPS before discovery for %s", rxMessage.Name)
			return
		}

		if globalSettings.DEBUG {
			log.Printf("GPS Received source:%d name:%s line:%s\n", thisGPS.gpsSource, rxMessage.Name, rxMessage.NmeaLine)
		}

		// Test NMEA sentence, and ignore if the CRC was wrong
		l_valid, validNMEAcs := common.ValidateNMEAChecksum(rxMessage.NmeaLine)
		if !validNMEAcs {
			if len(l_valid) > 0 {
				log.Printf("GPS error. Invalid NMEA string from [%s]: %s %s\n", rxMessage.Name, l_valid, rxMessage.NmeaLine)
			}
			thisGPS.gpsCRCErrors++
			return
		}

		// Always process OGN
		ognPublishNmea(rxMessage.NmeaLine)

		nmeaSlice := strings.Split(l_valid, ",")
		if slices.Contains(alwaysProcessTheseNMEAs(), nmeaSlice[0]) {
			// Some commands that do not affect GPS location/time services can always and should be processed
			processFlarmNmeaMessage(nmeaSlice)
			if ok := s.processNMEALine(l_valid, discoveredDevice); ok {
				registerSituationUpdate()
			}
		} else {

			// Process all NMEA for the current GPS
			if currentGPSName == rxMessage.Name {
				globalStatus.GPS_detected_type = discoveredDevice.GPSDetectedType | gps.GPS_PROTOCOL_NMEA
				globalStatus.GPS_source_name = rxMessage.Name
				globalStatus.GPS_connected = true
				globalStatus.GPS_source = uint(discoveredDevice.GPSSource)
				if ok := s.processNMEALine(l_valid, discoveredDevice); ok {
					registerSituationUpdate()
				}
			}

			// Test and remmeber this GPS fix quality so we can keep track of what GPS is suitable for fallback
			var situ SituationData
			situ.GPSFixQuality = 255 // We would not expect GPSFixQuality of 255, so we use it as a marker to see if it was changed
			situ, _ = parseNMEALine_GNGGA_GPGGA(nmeaSlice, situ)
			situ, _ = parseNMEALine_GNGSA_GPGSA_GLGSA_GAGSA_GBGSA(nmeaSlice, situ)
			if situ.GPSFixQuality != 255 {
				thisGPS.gpsFixQuality = situ.GPSFixQuality
				if thisGPS.gpsFixQuality > 0 {
					thisGPS.gpsLastGoodFix = stratuxClock.Time
				}
			}
		}
	}

	for {
		select {
		case rxMessage := <-s.rxMessageCh:
			processMessage(rxMessage)
		case currentGPSName = <-s.currentGPSNameCh:
		}
	}
}

/**
Watch the global current settings for a few variables and request a re-init of all GPS systems
when any of the globalSettings that are monitored are modified
*/
func (s *GPSDeviceManager) globalConfigChangeWatcher() {
	log.Printf("GPS: globalConfigChangeWatcher: Started")

	checkAndReInit := func() {
		g := s.settingsCopy.GPS_Enabled != globalSettings.GPS_Enabled
		x := s.settingsCopy.BleGPSEnabled != globalSettings.BleGPSEnabled
		n := s.settingsCopy.NetworkGPSEnabled != globalSettings.NetworkGPSEnabled
		y := s.settingsCopy.GPSPreferredSource != globalSettings.GPSPreferredSource
		d := s.settingsCopy.DEBUG != globalSettings.DEBUG
		b := !reflect.DeepEqual(s.settingsCopy.BleGPSAllowedDevices, globalSettings.BleGPSAllowedDevices)

		reInit := g || x || n || y || d || b

		if reInit {
			s.eh.Exit()
			s.settingsCopy = globalSettings

			// At this point new threads can start all adapters, but we assume here
			// that we will be faster resetting the GPS device
			s.gpsDeviceStatus.Clear()
			s.discoveredDevices.Clear()
			resetGPSGlobalStatus()
		}
	}

	timer := time.NewTicker(1000 * time.Millisecond)
	for {
		<-timer.C
		checkAndReInit()
	}
}

/**
Configure and enable GPS input sources
*/
func (s *GPSDeviceManager) configureGPSSubsystems() {
	s.eh.Add()
	defer s.eh.Done()
	s.ognTrackerConfigured = false // Bit of a hack to reset OGN detection here...
	defer func() {
		log.Printf("GPS: configureGPSSubsystems stopping")
		for _, o := range s.deviceList {
			o.Stop()
		}
		s.deviceList = make([]gpsDevice, 0)
		log.Printf("GPS: configureGPSSubsystems Stopped")
	}()

	if globalSettings.BleGPSEnabled {
		log.Printf("GPS: configureGPSSubsystems: Enable Bluetooth devices")
		bleGPSDevice := gps.NewBleGPSDevice(s.rxMessageCh)
		s.deviceList = append(s.deviceList, &bleGPSDevice)
		defer func() {
			// There is no need to read the settings back
			// globalSettings.BleGPSAllowedDevices = bleGPSDevice.GetConfig()
		}()
		go bleGPSDevice.Run(globalSettings.BleGPSAllowedDevices)
	}

	if globalSettings.GPS_Enabled {
		log.Printf("GPS: configureGPSSubsystems: Enable USB/Serial devices")
		serialGPSDevice := gps.NewSerialGPSDevice(s.rxMessageCh, globalSettings.DEBUG)
		s.deviceList = append(s.deviceList, &serialGPSDevice)	
		go serialGPSDevice.Run()
	}

	if globalSettings.NetworkGPSEnabled {
		log.Printf("GPS: configureGPSSubsystems: Enable Network devices")
		networkGPSDevice := gps.NewNetworkGPSDevice(s.rxMessageCh)
		s.deviceList = append(s.deviceList, &networkGPSDevice)	
		go networkGPSDevice.Run()
	}
	<-s.eh.C
}

func (s *GPSDeviceManager) ScanDevices() {
	// Since this is called from the UI, we prevent multiple requests to scan for devices
	if !s.scanMutex.TryLock() {
		return
	}

	s.eh.Add()
	leh := common.NewExitHelper();
	defer func() {
		globalStatus.GPS_Scanning = 0
		leh.Exit()
		s.eh.Done()
		log.Printf("GPS: ScanDevices: Done")
		s.scanMutex.Unlock()
	}()
	const TOTAL_SCAN_TIME_SEC = 30
	log.Printf("GPS: ScanDevices: Started")
	for _, o := range s.deviceList {
		go o.Scan(leh)
	}
	// Ticker to update the interface
	ticker := time.NewTicker(1 * time.Second)
	for globalStatus.GPS_Scanning = TOTAL_SCAN_TIME_SEC; globalStatus.GPS_Scanning > 0; {
		select {
		case <- s.eh.C:
			return
		case <- ticker.C:
			globalStatus.GPS_Scanning--	
		}
	}
}

func (s *GPSDeviceManager) Run() {
	Satellites = make(map[string]SatelliteInfo)
	log.Printf("GPS: Listen: Started")

	s.systemTimeSetter.Calibrate()
	go s.globalConfigChangeWatcher()
	go s.rxMessageHandler()
	go s.maintainPreferredGPSDevice()
	go s.handleDeviceDiscovery()
	for {
		s.configureGPSSubsystems()
	}
}
