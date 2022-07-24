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
	"math"
	"strconv"
	"strings"
	"sync"
	"time"

	"os/exec"

	"github.com/b3nn0/stratux/v2/common"
	"github.com/b3nn0/stratux/v2/gps"
	cmap "github.com/orcaman/concurrent-map"

)

// THis should never contain any NMEA codes for GPS Location data
var ALWAYS_PROCESS_NMEAS = []string{"PFLAU", "PFLAA", "POGNS", "POGNR", "POGNB"}

const GPS_FIX_TIME = 5000    // Time we expect a GPS satelite to have a valid fix TODO: RVT is there already a variable or time used for something like this?
const GPS_TIME_SOURCE = 2000 // Time we expect the GPS to be a valid source before we reconsider other GPS location sources
const FIX_QUALITY_3DGPS = 1  // 3DGPS
const FIX_QUALITY_AGPS = 2   // SBAS/WAAS

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
	gpsSource          uint16
	gpsFixQuality      uint8
	gpsLastSeen        uint64
	gpsLastGoodFix     uint64
	gpsCRCErrors       uint64
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

	systemTimeSetter    chan time.Time
	discoveredDevices   cmap.ConcurrentMap

}

func NewGPSDeviceManager() GPSDeviceManager {

	return GPSDeviceManager{
		gpsDeviceStatus:     make(map[string]GPSDeviceStatus),
		settingsCopy:        globalSettings,
		currentGPSName:      "",
		qh:                  common.NewQuitHelper(),
		m:                   sync.Mutex{},
		rxMessageCh:         make(chan gps.RXMessage, 20),
		txMessageCh:         make(chan gps.TXMessage, 20),
		discoveredDevicesCh: make(chan gps.DiscoveredDevice, 20),
		ognTrackerConfigured: false,
		systemTimeSetter:    make(chan time.Time, 0),
		discoveredDevices:   cmap.New(),
	}
}

/*
calcGPSAttitude estimates turn rate, pitch, and roll based on recent GPS groundspeed, track, and altitude / vertical speed.

Method uses stored performance statistics from myGPSPerfStats[]. Ideally, calculation is based on most recent 1.5 seconds of data,
assuming 10 Hz sampling frequency. Lower frequency sample rates will increase calculation window for smoother response, at the
cost of slightly increased lag.

(c) 2016 Keith Tschohl. All rights reserved.
Distributable under the terms of the "BSD-New" License that can be found in
the LICENSE file, herein included as part of this header.
*/

func calcGPSAttitude() bool {
	// check slice length. Return error if empty set or set zero values
	mySituation.muGPSPerformance.Lock()
	defer mySituation.muGPSPerformance.Unlock()
	length := len(myGPSPerfStats)
	index := length - 1

	if length == 0 {
		log.Printf("GPS attitude: No data received yet. Not calculating attitude.\n")
		return false
	} else if length == 1 {
		//log.Printf("myGPSPerfStats has one data point. Setting statistics to zero.\n")
		myGPSPerfStats[index].gpsTurnRate = 0
		myGPSPerfStats[index].gpsPitch = 0
		myGPSPerfStats[index].gpsRoll = 0
		return false
	}

	// check if GPS data was put in the structure more than three seconds ago -- this shouldn't happen unless something is wrong.
	if (stratuxClock.Milliseconds - myGPSPerfStats[index].stratuxTime) > 3000 {
		myGPSPerfStats[index].gpsTurnRate = 0
		myGPSPerfStats[index].gpsPitch = 0
		myGPSPerfStats[index].gpsRoll = 0
		log.Printf("GPS attitude: GPS data is more than three seconds old. Setting attitude to zero.\n")
		return false
	}

	// check time interval between samples
	t1 := myGPSPerfStats[index].nmeaTime
	t0 := myGPSPerfStats[index-1].nmeaTime
	dt := t1 - t0

	// first time error case: index is more than three seconds ahead of index-1
	if dt > 3 {
		log.Printf("GPS attitude: Can't calculate GPS attitude. Reference data is old. dt = %v\n", dt)
		return false
	}

	// second case: index is behind index-1. This could be result of day rollover. If time is within n seconds of UTC,
	// we rebase to the previous day, and will re-rebase the entire slice forward to the current day once all values roll over.
	//TODO: Validate by testing at 0000Z
	if dt < 0 {
		log.Printf("GPS attitude: Current GPS time (%.2f) is older than last GPS time (%.2f). Checking for 0000Z rollover.\n", t1, t0)
		if myGPSPerfStats[index-1].nmeaTime > 86300 && myGPSPerfStats[index].nmeaTime < 100 { // be generous with the time window at rollover
			myGPSPerfStats[index].nmeaTime += 86400
		} else {
			// time decreased, but not due to a recent rollover. Something odd is going on.
			log.Printf("GPS attitude: Time isn't near 0000Z. Unknown reason for offset. Can't calculate GPS attitude.\n")
			return false
		}

		// check time array to see if all timestamps are > 86401 seconds since midnight
		var tempTime []float64
		tempTime = make([]float64, length, length)
		for i := 0; i < length; i++ {
			tempTime[i] = float64(myGPSPerfStats[i].nmeaTime)
		}
		minTime, _ := common.ArrayMin(tempTime)
		if minTime > 86401.0 {
			log.Printf("GPS attitude: Rebasing GPS time since midnight to current day.\n")
			for i := 0; i < length; i++ {
				myGPSPerfStats[i].nmeaTime -= 86400
			}
		}

		// Verify adjustment
		dt = myGPSPerfStats[index].nmeaTime - myGPSPerfStats[index-1].nmeaTime
		log.Printf("GPS attitude: New dt = %f\n", dt)
		if dt > 3 {
			log.Printf("GPS attitude: Can't calculate GPS attitude. Reference data is old. dt = %v\n", dt)
			return false
		} else if dt < 0 {
			log.Printf("GPS attitude: Something went wrong rebasing the time.\n")
			return false
		}

	}

	// If all of the bounds checks pass, begin processing the GPS data.

	// local variables
	var headingAvg, dh, v_x, v_z, a_c, omega, slope, intercept float64
	var tempHdg, tempHdgUnwrapped, tempHdgTime, tempSpeed, tempVV, tempSpeedTime, tempRegWeights []float64 // temporary arrays for regression calculation
	var valid bool
	var lengthHeading, lengthSpeed int
	var halfwidth float64 // width of regression evaluation window. Minimum of 1.5 seconds and maximum of 3.5 seconds.

	center := float64(myGPSPerfStats[index].nmeaTime) // current time for calculating regression weights

	/*	// frequency detection
		tempSpeedTime = make([]float64, 0)
		for i := 1; i < length; i++ {
			dt = myGPSPerfStats[i].nmeaTime - myGPSPerfStats[i-1].nmeaTime
			if dt > 0.05 { // avoid double counting messages with same / similar timestamps
				tempSpeedTime = append(tempSpeedTime, float64(dt))
			}
		}
		//log.Printf("Delta time array is %v.\n",tempSpeedTime)
		dt_avg, valid = mean(tempSpeedTime)
		if valid && dt_avg > 0 {
			if globalSettings.DEBUG {
				log.Printf("GPS attitude: Average delta time is %.2f s (%.1f Hz)\n", dt_avg, 1/dt_avg)
			}
			halfwidth = 9 * dt_avg
			mySituation.GPSPositionSampleRate = 1 / dt_avg
		} else {
			if globalSettings.DEBUG {
				log.Printf("GPS attitude: Couldn't determine sample rate\n")
			}
			halfwidth = 3.5
			mySituation.GPSPositionSampleRate = 0
		}

		if halfwidth > 3.5 {
			halfwidth = 3.5 // limit calculation window to 3.5 seconds of data for 1 Hz or slower samples
		} else if halfwidth < 1.5 {
			halfwidth = 1.5 // use minimum of 1.5 seconds for sample rates faster than 5 Hz
		}
	*/
	halfwidth = calculateNavRate()

	//v_x = float64(myGPSPerfStats[index].gsf * 1.687810)
	//v_z = 0

	// first, parse groundspeed from RMC messages.
	tempSpeedTime = make([]float64, 0)
	tempSpeed = make([]float64, 0)
	tempRegWeights = make([]float64, 0)

	for i := 0; i < length; i++ {
		if myGPSPerfStats[i].msgType == "GPRMC" || myGPSPerfStats[i].msgType == "GNRMC" {
			tempSpeed = append(tempSpeed, float64(myGPSPerfStats[i].gsf))
			tempSpeedTime = append(tempSpeedTime, float64(myGPSPerfStats[i].nmeaTime))
			tempRegWeights = append(tempRegWeights, common.TriCubeWeight(center, halfwidth, float64(myGPSPerfStats[i].nmeaTime)))
		}
	}
	lengthSpeed = len(tempSpeed)
	if lengthSpeed == 0 {
		log.Printf("GPS Attitude: No groundspeed data could be parsed from NMEA RMC messages\n")
		return false
	} else if lengthSpeed == 1 {
		v_x = tempSpeed[0] * 1.687810
	} else {
		slope, intercept, valid = common.LinRegWeighted(tempSpeedTime, tempSpeed, tempRegWeights)
		if !valid {
			log.Printf("GPS attitude: Error calculating speed regression from NMEA RMC position messages")
			return false
		} else {
			v_x = (slope*float64(myGPSPerfStats[index].nmeaTime) + intercept) * 1.687810 // units are knots, converted to feet/sec
			//log.Printf("Avg speed %f calculated from %d RMC messages\n", v_x, lengthSpeed) // DEBUG
		}
	}

	// next, calculate vertical velocity from GGA altitude data.
	tempSpeedTime = make([]float64, 0)
	tempVV = make([]float64, 0)
	tempRegWeights = make([]float64, 0)

	for i := 0; i < length; i++ {
		if myGPSPerfStats[i].msgType == "GPGGA" || myGPSPerfStats[i].msgType == "GNGGA" {
			tempVV = append(tempVV, float64(myGPSPerfStats[i].alt))
			tempSpeedTime = append(tempSpeedTime, float64(myGPSPerfStats[i].nmeaTime))
			tempRegWeights = append(tempRegWeights, common.TriCubeWeight(center, halfwidth, float64(myGPSPerfStats[i].nmeaTime)))
		}
	}
	lengthSpeed = len(tempVV)
	if lengthSpeed < 2 {
		log.Printf("GPS Attitude: Not enough points to calculate vertical speed from NMEA GGA messages\n")
		return false
	} else {
		slope, _, valid = common.LinRegWeighted(tempSpeedTime, tempVV, tempRegWeights)
		if !valid {
			log.Printf("GPS attitude: Error calculating vertical speed regression from NMEA GGA messages")
			return false
		} else {
			v_z = slope // units are feet/sec
			//log.Printf("Avg VV %f calculated from %d GGA messages\n", v_z, lengthSpeed) // DEBUG
		}
	}

	// If we're going too slow for processNMEALine() to give us valid heading data, there's no sense in trying to parse it.
	// However, we need to return a valid level attitude so we don't get the "red X of death" on our AHRS display.
	// This will also eliminate most of the nuisance error message from the turn rate calculation.
	if v_x < 6 { // ~3.55 knots

		myGPSPerfStats[index].gpsPitch = 0
		myGPSPerfStats[index].gpsRoll = 0
		myGPSPerfStats[index].gpsTurnRate = 0
		myGPSPerfStats[index].gpsLoadFactor = 1.0
		mySituation.GPSTurnRate = 0

		// Output format:GPSAtttiude,seconds,nmeaTime,msg_type,GS,Course,Alt,VV,filtered_GS,filtered_course,turn rate,filtered_vv,pitch, roll,load_factor
		buf := fmt.Sprintf("GPSAttitude,%.1f,%.2f,%s,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", float64(stratuxClock.Milliseconds)/1000, myGPSPerfStats[index].nmeaTime, myGPSPerfStats[index].msgType, myGPSPerfStats[index].gsf, myGPSPerfStats[index].coursef, myGPSPerfStats[index].alt, myGPSPerfStats[index].vv, v_x/1.687810, headingAvg, myGPSPerfStats[index].gpsTurnRate, v_z, myGPSPerfStats[index].gpsPitch, myGPSPerfStats[index].gpsRoll, myGPSPerfStats[index].gpsLoadFactor)
		if globalSettings.DEBUG {
			log.Printf("%s", buf) // FIXME. Send to sqlite log or other file?
		}
		logGPSAttitude(myGPSPerfStats[index])
		//replayLog(buf, MSGCLASS_AHRS)

		return true
	}

	// Heading.  Same method used for UBX and generic.
	// First, walk through the PerfStats and parse only valid heading data.
	//log.Printf("Raw heading data:")
	for i := 0; i < length; i++ {
		//log.Printf("%.1f,",myGPSPerfStats[i].coursef)
		if myGPSPerfStats[i].coursef >= 0 { // negative values are used to flag invalid / unavailable course
			tempHdg = append(tempHdg, float64(myGPSPerfStats[i].coursef))
			tempHdgTime = append(tempHdgTime, float64(myGPSPerfStats[i].nmeaTime))
		}
	}
	//log.Printf("\n")
	//log.Printf("tempHdg: %v\n", tempHdg)

	// Next, unwrap the heading so we don't mess up the regression by fitting a line across the 0/360 deg discontinuity.
	lengthHeading = len(tempHdg)
	tempHdgUnwrapped = make([]float64, lengthHeading, lengthHeading)
	tempRegWeights = make([]float64, lengthHeading, lengthHeading)

	if lengthHeading > 1 {
		tempHdgUnwrapped[0] = tempHdg[0]
		tempRegWeights[0] = common.TriCubeWeight(center, halfwidth, tempHdgTime[0])
		for i := 1; i < lengthHeading; i++ {
			tempRegWeights[i] = common.TriCubeWeight(center, halfwidth, tempHdgTime[i])
			if math.Abs(tempHdg[i]-tempHdg[i-1]) < 180 { // case 1: if angle change is less than 180 degrees, use the same reference system
				tempHdgUnwrapped[i] = tempHdgUnwrapped[i-1] + tempHdg[i] - tempHdg[i-1]
			} else if tempHdg[i] > tempHdg[i-1] { // case 2: heading has wrapped around from NE to NW. Subtract 360 to keep consistent with previous data.
				tempHdgUnwrapped[i] = tempHdgUnwrapped[i-1] + tempHdg[i] - tempHdg[i-1] - 360
			} else { // case 3:  heading has wrapped around from NW to NE. Add 360 to keep consistent with previous data.
				tempHdgUnwrapped[i] = tempHdgUnwrapped[i-1] + tempHdg[i] - tempHdg[i-1] + 360
			}
		}
	} else { //
		if globalSettings.DEBUG {
			log.Printf("GPS attitude: Can't calculate turn rate with less than two points.\n")
		}
		return false
	}

	// Finally, calculate turn rate as the slope of the weighted linear regression of unwrapped heading.
	slope, intercept, valid = common.LinRegWeighted(tempHdgTime, tempHdgUnwrapped, tempRegWeights)

	if !valid {
		log.Printf("GPS attitude: Regression error calculating turn rate")
		return false
	} else {
		headingAvg = slope*float64(myGPSPerfStats[index].nmeaTime) + intercept
		dh = slope // units are deg per sec; no conversion needed here
		//log.Printf("Calculated heading and turn rate: %.3f degrees, %.3f deg/sec\n",headingAvg,dh)
	}

	myGPSPerfStats[index].gpsTurnRate = dh
	mySituation.GPSTurnRate = dh

	// pitch angle -- or to be more pedantic, glide / climb angle, since we're just looking a rise-over-run.
	// roll angle, based on turn rate and ground speed. Only valid for coordinated flight. Differences between airspeed and groundspeed will trip this up.
	if v_x > 20 { // reduce nuisance 'bounce' at low speeds. 20 ft/sec = 11.9 knots.
		myGPSPerfStats[index].gpsPitch = math.Atan2(v_z, v_x) * 180.0 / math.Pi

		/*
			Governing equations for roll calculations

			Physics tells us that
				a_z = g     (in steady-state flight -- climbing, descending, or level -- this is gravity. 9.81 m/s^2 or 32.2 ft/s^2)
				a_c = v^2/r (centripetal acceleration)

			We don't know r. However, we do know the tangential velocity (v) and angular velocity (omega). Express omega in radians per unit time, and

				v = omega*r

			By substituting and rearranging terms:

				a_c = v^2 / (v / omega)
				a_c = v*omega

			Free body diagram time!

				   /|
			  a_r / |  a_z
				 /__|
			   X   a_c
				\_________________ [For the purpose of this comment, " X" is an airplane in a 20 degree bank. Use your imagination, mkay?)

			Resultant acceleration a_r is what the wings feel; a_r/a_z = load factor. Anyway, trig out the bank angle:

				bank angle = atan(a_c/a_z)
						   = atan(v*omega/g)

				wing loading = sqrt(a_c^2 + a_z^2) / g

		*/

		g := 32.174                                               // ft/(s^2)
		omega = common.Radians(myGPSPerfStats[index].gpsTurnRate) // need radians/sec
		a_c = v_x * omega
		myGPSPerfStats[index].gpsRoll = math.Atan2(a_c, g) * 180 / math.Pi // output is degrees
		myGPSPerfStats[index].gpsLoadFactor = math.Sqrt(a_c*a_c+g*g) / g
	} else {
		myGPSPerfStats[index].gpsPitch = 0
		myGPSPerfStats[index].gpsRoll = 0
		myGPSPerfStats[index].gpsLoadFactor = 1
	}

	if globalSettings.DEBUG {
		// Output format:GPSAtttiude,seconds,nmeaTime,msg_type,GS,Course,Alt,VV,filtered_GS,filtered_course,turn rate,filtered_vv,pitch, roll,load_factor
		buf := fmt.Sprintf("GPSAttitude,%.1f,%.2f,%s,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", float64(stratuxClock.Milliseconds)/1000, myGPSPerfStats[index].nmeaTime, myGPSPerfStats[index].msgType, myGPSPerfStats[index].gsf, myGPSPerfStats[index].coursef, myGPSPerfStats[index].alt, myGPSPerfStats[index].vv, v_x/1.687810, headingAvg, myGPSPerfStats[index].gpsTurnRate, v_z, myGPSPerfStats[index].gpsPitch, myGPSPerfStats[index].gpsRoll, myGPSPerfStats[index].gpsLoadFactor)
		log.Printf("%s", buf) // FIXME. Send to sqlite log or other file?
	}

	logGPSAttitude(myGPSPerfStats[index])
	//replayLog(buf, MSGCLASS_AHRS)
	return true
}

/*
	registerSituationUpdate().
	 Called whenever there is a change in mySituation.
*/
func registerSituationUpdate() {
	logSituation()
	situationUpdate.SendJSON(mySituation)
}

func calculateNavRate() float64 {
	length := len(myGPSPerfStats)
	tempSpeedTime := make([]float64, 0)

	for i := 1; i < length; i++ {
		dt := myGPSPerfStats[i].nmeaTime - myGPSPerfStats[i-1].nmeaTime
		if dt > 0.05 { // avoid double counting messages with same / similar timestamps
			tempSpeedTime = append(tempSpeedTime, float64(dt))
		}
	}

	var halfwidth float64
	dt_avg, valid := common.Mean(tempSpeedTime)
	if valid && dt_avg > 0 {
		if globalSettings.DEBUG {
			log.Printf("GPS attitude: Average delta time is %.2f s (%.1f Hz)\n", dt_avg, 1/dt_avg)
		}
		halfwidth = 9 * dt_avg
		mySituation.GPSPositionSampleRate = 1 / dt_avg
	} else {
		if globalSettings.DEBUG {
			log.Printf("GPS attitude: Couldn't determine sample rate\n")
		}
		halfwidth = 3.5
		mySituation.GPSPositionSampleRate = 0
	}

	if halfwidth > 3.5 {
		halfwidth = 3.5 // limit calculation window to 3.5 seconds of data for 1 Hz or slower samples
	} else if halfwidth < 1.5 {
		halfwidth = 1.5 // use minimum of 1.5 seconds for sample rates faster than 5 Hz
	}

	return halfwidth
}

/*
	updateConstellation(): Periodic cleanup and statistics calculation for 'Satellites'
		data structure. Calling functions must protect this in a mySituation.muSatellite.

*/

func updateConstellation() {
	var sats, tracked, seen uint8
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

	mySituation.GPSSatellites = uint16(sats)
	mySituation.GPSSatellitesTracked = uint16(tracked)
	mySituation.GPSSatellitesSeen = uint16(seen)
}

/**
  Update satelite information from GPS data
  This should be used from GNGSA,GPGSA,GLGSA,GAGSA,GBGSA sentences only
*/
func updateSatellites(x []string) {

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

		if err != nil {
			if globalSettings.DEBUG {
				log.Printf("Failed to get SV type %s\n", err.Error())
			}
		}

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

func updateGPSPerfmStat(thisGpsPerf gpsPerfStats) {
	mySituation.muGPSPerformance.Lock()
	myGPSPerfStats = append(myGPSPerfStats, thisGpsPerf)
	lenGPSPerfStats := len(myGPSPerfStats)
	//	log.Printf("GPSPerf array has %n elements. Contents are: %v\n",lenGPSPerfStats,myGPSPerfStats)
	if lenGPSPerfStats > 299 { //30 seconds @ 10 Hz for UBX, 30 seconds @ 5 Hz for MTK or SIRF with 2x messages per 200 ms)
		myGPSPerfStats = myGPSPerfStats[(lenGPSPerfStats - 299):] // remove the first n entries if more than 300 in the slice
	}
	mySituation.muGPSPerformance.Unlock()
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Process a NMEA line and update strauc
//////////////////////////////////////////////////////////////////////////////////////////////////////

func (s *GPSDeviceManager) processNMEALine(l string, name string, gpsTimeOffsetPpsMs time.Duration) (sentenceUsed bool) {

	mySituation.muGPS.Lock()
	defer func() {
		if sentenceUsed || globalSettings.DEBUG {
			registerSituationUpdate()
		}
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

	ognPublishNmea(l)
	x := strings.Split(l[1:], ",")

	mySituation.GPSLastValidNMEAMessageTime = stratuxClock.Time
	mySituation.GPSLastValidNMEAMessage = l

	// Use a unbuffered channel to set the time calling exec,
	// this span's a new process which can take a long time to return When setting time we ensure we just ensure we only call this once

	// ############################################# GNVTG GNVTG #############################################
	if (x[0] == "GNVTG") || (x[0] == "GPVTG") { // Ground track information.
		data, err := parseNMEALine_GNVTG_GPVTG(x, &mySituation)
		if err == nil {
			// TODO: RVT  decide how to correctly lock mySituation for writing
			mySituation = data
		}
		return err == nil
		// ############################################# GNGGA GPGGA #############################################
	} else if (x[0] == "GNGGA") || (x[0] == "GPGGA") { // Position fix.
		data, err := parseNMEALine_GNGGA_GPGGA(x, &mySituation)
		if err == nil {
			// TODO: RVT  decide how to correctly lock mySituation for writing
			mySituation = data

			thisGpsPerf.nmeaTime = mySituation.GPSLastFixSinceMidnightUTC
			thisGpsPerf.alt = float32(mySituation.GPSAltitudeMSL)
			thisGpsPerf.msgType = x[0]
			updateGPSPerfmStat(thisGpsPerf)
		}
		return err == nil
		// ############################################# GNRMC GPRMC #############################################
	} else if (x[0] == "GNRMC") || (x[0] == "GPRMC") { // Recommended Minimum data.
		data, err := parseNMEALine_GNRMC_GPRMC(x, &mySituation, gpsTimeOffsetPpsMs)
		if err == nil {
			previousSituation := mySituation
			// TODO: RVT  decide how to correctly lock mySituation for writing
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

			if len(x[9]) == 6 {
				if time.Since(mySituation.GPSTime) > 300*time.Millisecond || time.Since(mySituation.GPSTime) < -300*time.Millisecond {
					select {
					case s.systemTimeSetter <- mySituation.GPSTime:
					default:
						if globalSettings.DEBUG {
							log.Println("WARNING: Time setting in progress, disregarding value")
						}
					}
				}
			}

			updateGPSPerfmStat(thisGpsPerf)
			stratuxClock.SetRealTimeReference(mySituation.GPSTime)
			setDataLogTimeWithGPS(mySituation)
		}
		return err == nil
		// ############################################# GNGSA GPGSA GLGSA GAGSA GBGSA #############################################
		// TODO: RVT : In original GPS code we only do GNGSA and GLGSA, why do we now do more??
	} else if (x[0] == "GNGSA") || (x[0] == "GPGSA") || (x[0] == "GLGSA") || (x[0] == "GAGSA") || (x[0] == "GBGSA") { // Satellite data.
		data, err := parseNMEALine_GNGSA_GPGSA_GLGSA_GAGSA_GBGSA(x, &mySituation)
		if err == nil {
			// TODO: RVT  decide how to correctly lock mySituation for writing
			mySituation = data
			hdop := mySituation.GPSHorizontalAccuracy
			if mySituation.GPSFixQuality == FIX_QUALITY_AGPS { // Rough 95% confidence estimate for SBAS solution
				if globalStatus.GPS_detected_type == common.GPS_TYPE_UBX9 {
					mySituation.GPSHorizontalAccuracy = float32(hdop * 3.0) // ublox 9
				} else {
					mySituation.GPSHorizontalAccuracy = float32(hdop * 4.0) // ublox 6/7/8
				}
			} else { // Rough 95% confidence estimate non-SBAS solution
				if globalStatus.GPS_detected_type == common.GPS_TYPE_UBX9 {
					mySituation.GPSHorizontalAccuracy = float32(hdop * 4.0) // ublox 9
				} else {
					mySituation.GPSHorizontalAccuracy = float32(hdop * 5.0) // ublox 6/7/8
				}
			}
			mySituation.muSatellite.Lock()
			updateConstellation()
			updateSatellites(x) // TODO: RVT Shift left x with 3 positions so it will be 0..12
			mySituation.muSatellite.Unlock()
		}
		return err == nil
		// ############################################# GPGSV GLGSV GAGSV GBGSV #############################################
	} else if (x[0] == "GPGSV") || (x[0] == "GLGSV") || (x[0] == "GAGSV") || (x[0] == "GBGSV") { // GPS + SBAS or GLONASS or Galileo or Beidou satellites in view message.
		data, err := parseNMEALine_GPGSV_GLGSV_GAGSV_GBGSV(x, &mySituation)
		if err == nil {
			// TODO: RVT  decide how to correctly lock mySituation for writing
			mySituation = data
			mySituation.muSatellite.Lock()
			updateConstellation()
			updateSatellitesInView(x)
			mySituation.muSatellite.Unlock()

		}
		return err == nil
		// ############################################# POGNB #############################################
	} else if x[0] == "POGNB" {
		data, err := parseNMEALine_POGNB(x, &mySituation)
		if err == nil {
			mySituation.muBaro.Lock()
			// TODO: RVT  decide how to correctly lock mySituation for writing
			mySituation = data
			mySituation.muBaro.Unlock()
		}
		return err == nil
		// ############################################# POGNR #############################################
	} else if x[0] == "POGNR" {
		// Only sent by OGN tracker. We use this to detect that OGN tracker is connected and configure it as needed
		if !s.ognTrackerConfigured {
			s.ognTrackerConfigured = true
			s.requestOgnTrackerConfiguration(name)
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
	} else if x[0] == "PGRMZ" && ((globalStatus.GPS_detected_type&0x0f) == common.GPS_TYPE_SERIAL || (globalStatus.GPS_detected_type&0x0f) == common.GPS_TYPE_SOFTRF_DONGLE) {

		fq := SituationData{}
		data, err := parseNMEALine_PGRMZ(x, &fq)

		if err != nil && (!isTempPressValid() || (mySituation.BaroSourceType != common.BARO_TYPE_BMP280 && mySituation.BaroSourceType != common.BARO_TYPE_OGNTRACKER)) {
			mySituation.muBaro.Lock()
			mySituation.BaroPressureAltitude = data.BaroPressureAltitude // meters to feet
			mySituation.BaroLastMeasurementTime = data.BaroLastMeasurementTime
			mySituation.BaroSourceType = data.BaroSourceType
			mySituation.muBaro.Unlock()
		}

		return err != nil
	}

	// If we've gotten this far, the message isn't one that we can use.
	return false
}

func isGPSConnected() bool {
	return stratuxClock.Since(mySituation.GPSLastValidNMEAMessageTime) < 5*time.Second
}

/*
isGPSValid returns true only if a valid position fix has been seen in the last 3 seconds,
and if the GPS subsystem has recently detected a GPS device.

If false, 'GPSFixQuality` is set to 0 ("No fix"), as is the number of satellites in solution.
*/

func setGPSNotValidMySituation() {
	mySituation.GPSFixQuality = 0
	mySituation.GPSSatellites = 0
	mySituation.GPSHorizontalAccuracy = 999999
	mySituation.GPSVerticalAccuracy = 999999
	mySituation.GPSNACp = 0
}

/** Reset GOS global status
*/
func resetGPSGlobalStatus() {
	// Reset global status
	globalStatus.GPS_satellites_locked = 0x00
	globalStatus.GPS_satellites_seen = 0x00
	globalStatus.GPS_satellites_tracked = 0x00
	globalStatus.GPS_position_accuracy = 99999
	globalStatus.GPS_connected = false
	globalStatus.GPS_source = 0x00
	globalStatus.GPS_solution = ""
	globalStatus.GPS_detected_type = 0x00
	globalStatus.GPS_NetworkRemoteIp = ""

	mySituation.GPSSatellitesSeen = 0x00
	mySituation.GPSSatellitesTracked = 0x00

	// reset my situation
	setGPSNotValidMySituation()
	updateStatus()

	// Remove all satellites
	Satellites = make(map[string]SatelliteInfo)
}

func isGPSValid() bool {
	isValid := false
	if (stratuxClock.Since(mySituation.GPSLastFixLocalTime) < 3*time.Second) && globalStatus.GPS_connected && mySituation.GPSFixQuality > 0 {
		isValid = true
	} else {
		setGPSNotValidMySituation()
	}
	return isValid
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
	s.configureGPS( gps.TXMessage {
		Message: []byte(common.AppendNmeaChecksum("$POGNS,NavRate=5") + "\r\n"),
		Name: name,
	})

	// Request stored OGN tracker configuration to be send back to us
	s.configureGPS( gps.TXMessage {
		Message: []byte(s.getOgnTrackerConfigQueryString()),
		Name: name,
	})

	device,_ := s.gpsDeviceStatus[name]
	s.gpsDeviceStatus[name] = device
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

	s.configureGPS( gps.TXMessage {
		Message: []byte(cfg),
		Name: name,
	})

	s.configureGPS( gps.TXMessage {
		Message: []byte(s.getOgnTrackerConfigQueryString()),
		Name: name,
	})
}

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

//////////////////////////////////////////////////////////////////////////////////////////////////////
// OGN
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
// GPSDeviceStatus
//////////////////////////////////////////////////////////////////////////////////////////////////////

/**
Returns true when any device has a valid fix
*/
func (d *GPSDeviceStatus) hasValidFix() bool {
	return d.gpsFixQuality > 0 &&
		d.gpsLastSeen > stratuxClock.Milliseconds-GPS_TIME_SOURCE &&
		d.gpsLastGoodFix > stratuxClock.Milliseconds-GPS_FIX_TIME
}

// Find a gps Source with a fix
// Not thread safe, only call from within a lock
func (s *GPSDeviceManager) gpsDeviceWithFix(GpsSource uint16) (string, GPSDeviceStatus) {
	// TODO: RVT: We should sort on a GPS devices with the best fix ??
	for k, v := range s.gpsDeviceStatus {
		if v.gpsSource == GpsSource && v.hasValidFix() {
			return k, v
		}
	}
	return "", GPSDeviceStatus{}
}

// Find any GPS with a fix with preference to gpsSource
// Not thread safe, only call from within a lock
func (s *GPSDeviceManager) anyGpsDeviceWithFix(gpsSource uint16) (string, GPSDeviceStatus) {
	k, v := s.gpsDeviceWithFix(gpsSource)
	if !v.hasValidFix() {
		k, v = s.gpsDeviceWithFix(0)
	}
	return k, v
}

/**
Maintain and decide on what gps source to use
When we do not have any GPS, we just pick one from the list that has a fix, if not we just pick the first if any
**/
func (s *GPSDeviceManager) maintainAndDecideGPSConnections() {
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
			// GPS with a good fix, if not found then we keep current GPS
			GPSPreferredSource := uint16(globalSettings.GPSPreferredSource)
			if !currentGPSSource.hasValidFix() {
				anyGpsName, _ := s.anyGpsDeviceWithFix(GPSPreferredSource)
				if anyGpsName != "" {
					s.currentGPSName = anyGpsName
				}
			}

			// Verify if the current GPS source might not be preferred, if so then lookup a GPS
			// source that is preferrred and has a fix, if found use the preferred source if not leave it as is
			if currentGPSSource.gpsSource != GPSPreferredSource {
				possiblePreferredSource, _ := s.gpsDeviceWithFix(GPSPreferredSource)
				if possiblePreferredSource != "" {
					s.currentGPSName = possiblePreferredSource
				}
			}
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
				gpsFixQuality:      0,
				gpsLastGoodFix:     0,
				gpsCRCErrors:       0,
			}
		}
		thisGPS.gpsLastSeen = stratuxClock.Milliseconds

		// Validate NMEA sentence, and ignore if the CRC was wrong
		l_valid, validNMEAcs := common.ValidateNMEAChecksum(rxMessage.NmeaLine)
		if !validNMEAcs {
			if len(l_valid) > 0 {
				log.Printf("GPS error. Invalid NMEA string: %s %s\n", l_valid, rxMessage.NmeaLine) // remove log message once validation complete
			}
			thisGPS.gpsCRCErrors++
		} else {
			globalStatus.GPS_detected_type |= common.GPS_PROTOCOL_NMEA
			nmeaSlice := strings.Split(l_valid, ",")

			// We check, however we always assume we always get a discvery message before the first NMEA message for a fresh attached GPS
			if deviceRaw, ok := s.discoveredDevices.Get(rxMessage.Name); ok {
				discoveredDevice := deviceRaw.(gps.DiscoveredDevice)

				if common.StringInSlice(nmeaSlice[0], ALWAYS_PROCESS_NMEAS) {
					// Some commands that do not affect GPS location services can always and should bebe processed
					processFlarmNmeaMessage(nmeaSlice)
					s.processNMEALine(rxMessage.NmeaLine, rxMessage.Name, discoveredDevice.GpsTimeOffsetPpsMs)
				} else {

					// Process all NMEA for the current GPS
					if s.currentGPSName == rxMessage.Name { 
						s.processNMEALine(rxMessage.NmeaLine, rxMessage.Name, discoveredDevice.GpsTimeOffsetPpsMs)
						globalStatus.GPS_source_name = rxMessage.Name
						globalStatus.GPS_connected = true
						globalStatus.GPS_detected_type = (globalStatus.GPS_detected_type & 0xF0) | discoveredDevice.GpsDetectedType
						globalStatus.GPS_source = uint(discoveredDevice.GpsSource)
						thisGPS.gpsSource = discoveredDevice.GpsSource // TODO: RVT: We need to gpsSource in the gpsDevice status for sorting, we could also just use the map?
					}
	
					// Check and remmeber this GPS fix quality
					fq := SituationData{}
					fq.GPSFixQuality = 255 // We would not expect a fq of 255, so we use it as a marker to see if it was changed
					situ, _ := parseNMEALine_GNGGA_GPGGA(nmeaSlice, &fq)
					situ, _ = parseNMEALine_GNGSA_GPGSA_GLGSA_GAGSA_GBGSA(nmeaSlice, &situ)
					if situ.GPSFixQuality != 255 {
						thisGPS.gpsFixQuality = situ.GPSFixQuality
						if thisGPS.gpsFixQuality > 0 {
							thisGPS.gpsLastGoodFix = stratuxClock.Milliseconds
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
			s.settingsCopy = globalSettings

			s.qh.Quit()
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
Configure and enable GPS system
*/
func (s *GPSDeviceManager) configureGPSSubsystems() {
	log.Printf("GPS: configureGPSSubsystems: Started")
	s.qh.Add()
	defer s.qh.Done()

	bleGPSDevice := gps.NewBleGPSDevice(s.rxMessageCh, s.discoveredDevicesCh)
	serialGPSDevice := gps.NewSerialGPSDevice(s.rxMessageCh, s.discoveredDevicesCh, globalSettings.DEBUG)

	defer func() {
		log.Printf("GPS: configureGPSSubsystems stopping")
		bleGPSDevice.Stop()
		serialGPSDevice.Stop()
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

	<-s.qh.C
}

func (s *GPSDeviceManager) maintainConnectedDeviceList() {
	handleDeviceDiscovery := func(discoveredDevice gps.DiscoveredDevice) {

		// Copy TXChannel if one previously was already known because device discovery might not always have a TXCHannel added
		if previousRaw, ok := s.discoveredDevices.Get(discoveredDevice.Name); ok {
			previous := previousRaw.(gps.DiscoveredDevice)
			if previous.HasTXChannel && !discoveredDevice.HasTXChannel {
				discoveredDevice.HasTXChannel = true
				discoveredDevice.TXChannel = previous.TXChannel
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
				Name:            v.Name,
				Connected:       v.Connected,
				LastDiscoveryMessage:        v.LastDiscoveryMessage,
				GpsDetectedType: v.GpsDetectedType,
				GpsSource:       v.GpsSource,
			})
		}
		globalStatus.GPS_Discovery = deviceList
	}

	sendGPSMessage := func(message *gps.TXMessage) {
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
			handleDeviceDiscovery(discoveredDevice)
			break
		case txConfig := <-s.txMessageCh:
			sendGPSMessage(&txConfig)
			break
		}
	}
}

/**
goroutine that listens to systemTimeSetter channel and set the time accordingly
*/
func (s *GPSDeviceManager) systemTimeSetterHandler() {
	for {
		newTime := <-s.systemTimeSetter
		setStr := newTime.Format("20060102 15:04:05.000") + " UTC"
		log.Printf("setting system time from %s to: '%s'\n", time.Now().Format("20060102 15:04:05.000"), setStr)

		var err error
		if common.IsRunningAsRoot() {
			err = exec.Command("date", "-s", setStr).Run()
		} else {
			err = exec.Command("sudo", "date", "-s", setStr).Run()
		}
		if err != nil {
			log.Printf("Set Date failure: %s error\n", err)
		} else {
			log.Printf("Time set from GPS. Current time is %v\n", time.Now())
		}
	}
}

func (s *GPSDeviceManager) Run() {
	Satellites = make(map[string]SatelliteInfo)
	log.Printf("GPS: Listen: Started")
	// message := make(chan DiscoveredDevice, 20)

	go s.systemTimeSetterHandler()
	go s.globalConfigChangeWatcher()
	go s.rxMessageHandler()
	go s.maintainAndDecideGPSConnections()
	go s.maintainConnectedDeviceList()
	for {
		s.configureGPSSubsystems()
		time.Sleep(1000 * time.Millisecond)
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// GPSDeviceStatus
//////////////////////////////////////////////////////////////////////////////////////////////////////
