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
	"time"

	"github.com/b3nn0/stratux/v2/common"
)

var gnssBaroAltDiffs = make(map[int]int)


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

// Little helper function to dump the gnssBaroAltDiffs map to CSV for plotting
//func dumpValues() {
//	vals := ""
//	for k, v := range gnssBaroAltDiffs {
//		vals += fmt.Sprintf("%d,%d\n", k*100, v)
//	}
//	ioutil.WriteFile("/tmp/values.csv", []byte(vals), 0644)
//}

// Maps 100ft bands to gnssBaroAltDiffs of known traffic.
// This will then be used to estimate our own baro altitude from GNSS if we don't have a pressure sensor connected...
// Sometimes dump1090 will deliver some strange invalid data with wild values, so we need some outlier detection.
// To achieve that, the algorithm works like this:
// 1. Create a linear regression over all confirmed targets altitude->GnssBaroDiff mapping
// 2. filter out targets that are more than +-400ft off from that regression
// 3. Now for the remaining targets, sort them into buckets again in a smoothed out way
// 4. Use a weighted linear regression with a higher weight around our own altitude to determine a smoothed-out GnssBaroDiff for our current altitude
// 5. Use GPS Alt +- GnssBaroDiff to determine our own baro alt

func BaroAltGuesser() {
	ticker := time.NewTicker(1 * time.Second)
	for {
		<-ticker.C
		// Create linear regression from GnssBaroAltDiffs we have confirmed already
		var alts, diffs []float64
		for k, v := range gnssBaroAltDiffs {
			alts = append(alts, float64(k*100))
			diffs = append(diffs, float64(v))
		}
		slope, intercept, valid := common.LinReg(alts, diffs)
		//fmt.Printf("General: %f * x + %f \n", slope, intercept)

		trafficMutex.Lock()
		for _, ti := range traffic {
			if ti.ReceivedMsgs < 30 || ti.SignalLevel < -28 || ti.SignalLevel > -3 {
				continue // Make sure it is actually a confirmed target, so we don't accidentally use invalid values from invalid data
			}
			if stratuxClock.Since(ti.Last_GnssDiff) > 1*time.Second || ti.Alt <= 1 || stratuxClock.Since(ti.Last_alt) > 1*time.Second {
				continue // already considered this value or we don't have a value - skip
			}

			bucket := int(ti.Alt / 100)
			if bucket <= 0 {
				continue // sometimes some random altitude reports - usually close to 0ft but GNSS diff from around 40000.. try to filter those
			}

			if len(gnssBaroAltDiffs) >= 30 && valid {
				// Check if this ti is potentially an outlier/invalid data..
				estimatedDiff := float64(ti.Alt)*slope + intercept
				if math.Abs(float64(ti.GnssDiffFromBaroAlt)-estimatedDiff) > 400 {
					fmt.Printf("Ignoring %d, alt=%d for baro computation. Expected GnssDiff: %f, Received: %d \n", ti.Icao_addr, ti.Alt, estimatedDiff, ti.GnssDiffFromBaroAlt)
					continue
				}
			}

			if val, ok := gnssBaroAltDiffs[bucket]; ok {
				// weighted average - don't tune too quickly... smooth over one minute (for one aircraft, half a minute for two, etc).
				gnssBaroAltDiffs[bucket] = (val*59 + int(ti.GnssDiffFromBaroAlt)*1) / 60
			} else {
				gnssBaroAltDiffs[bucket] = int(ti.GnssDiffFromBaroAlt)
			}
		}
		trafficMutex.Unlock()

		if len(gnssBaroAltDiffs) < 30 {
			continue // not enough data
		}
		if isGPSValid() && (!isTempPressValid() || mySituation.BaroSourceType == BARO_TYPE_NONE || mySituation.BaroSourceType == BARO_TYPE_ADSBESTIMATE) {
			// We have no real baro source.. try to estimate baro altitude with the help of closeby ADS-B aircraft that define BaroGnssDiff...

			myAlt := mySituation.GPSAltitudeMSL
			if isTempPressValid() {
				myAlt = mySituation.BaroPressureAltitude // we have something better than GPS from a previous run or something
			}
			alts := make([]float64, 0, len(gnssBaroAltDiffs))
			diffs := make([]float64, 0, len(gnssBaroAltDiffs))
			weights := make([]float64, 0, len(gnssBaroAltDiffs)) // Weigh close altitudes higher than far altitudes for linreg
			for k, v := range gnssBaroAltDiffs {
				bucketAlt := float64(k*100 + 50)
				alts = append(alts, bucketAlt) // Compute back from bucket to "real" altitude (+50 to be in the center of the bucket)
				diffs = append(diffs, float64(v))
				// Weight: 1 / altitudeDifference / 100
				weight := math.Abs(float64(myAlt) - bucketAlt)
				if weight == 0 {
					weight = 1
				} else {
					weight = math.Min(1/(weight/1000), 1)
				}
				weights = append(weights, weight*5) // 5 = arbitrary factor to weight the local data even stronger compared to stuff thats 30000ft above.
				// See https://www.desmos.com/calculator/qiqmb4wrev for why this seems to make sense.
				// X-axis is altitude, Y axis is reported GnssBaroDiff
			}
			if len(gnssBaroAltDiffs) >= 2 {
				slope, intercept, valid := common.LinRegWeighted(alts, diffs, weights)
				if valid {
					gnssBaroDiff := float64(myAlt)*slope + intercept
					mySituation.muBaro.Lock()
					mySituation.BaroLastMeasurementTime = stratuxClock.Time
					mySituation.BaroPressureAltitude = mySituation.GPSHeightAboveEllipsoid - float32(gnssBaroDiff)
					mySituation.BaroSourceType = BARO_TYPE_ADSBESTIMATE
					//fmt.Printf(" %f * x + %f \n", slope, intercept)
					mySituation.muBaro.Unlock()
				}
			}
		}
	}
}

/*
	ffAttitudeSender()
	 Send AHRS message in FF format every 200ms.
*/

func FFAttitudeSender() {
	ticker := time.NewTicker(200 * time.Millisecond)
	for {
		<-ticker.C
		makeFFAHRSMessage()
	}
}

/*

	ForeFlight "AHRS Message".

	Sends AHRS information to ForeFlight.

*/

func makeFFAHRSMessage() {
	msg := make([]byte, 12)
	msg[0] = 0x65 // Message type "ForeFlight".
	msg[1] = 0x01 // AHRS message identifier.

	// Values if invalid
	pitch := int16(0x7FFF)
	roll := int16(0x7FFF)
	hdg := uint16(0xFFFF)
	ias := uint16(0xFFFF)
	tas := uint16(0xFFFF)

	if isAHRSValid() {
		if !isAHRSInvalidValue(mySituation.AHRSPitch) {
			pitch = common.RoundToInt16(mySituation.AHRSPitch * 10)
		}
		if !isAHRSInvalidValue(mySituation.AHRSRoll) {
			roll = common.RoundToInt16(mySituation.AHRSRoll * 10)
		}
	}

	// Roll.
	msg[2] = byte((roll >> 8) & 0xFF)
	msg[3] = byte(roll & 0xFF)

	// Pitch.
	msg[4] = byte((pitch >> 8) & 0xFF)
	msg[5] = byte(pitch & 0xFF)

	// Heading.
	msg[6] = byte((hdg >> 8) & 0xFF)
	msg[7] = byte(hdg & 0xFF)

	// Indicated Airspeed.
	msg[8] = byte((ias >> 8) & 0xFF)
	msg[9] = byte(ias & 0xFF)

	// True Airspeed.
	msg[10] = byte((tas >> 8) & 0xFF)
	msg[11] = byte(tas & 0xFF)

	sendMsg(prepareMessage(msg), NETWORK_AHRS_GDL90, 200*time.Millisecond, 3)
}

func GPSAttitudeSender() {
	timer := time.NewTicker(100 * time.Millisecond) // ~10Hz update.
	for {
		<-timer.C
		if !(globalStatus.GPS_connected || globalStatus.IMUConnected) {
			myGPSPerfStats = make([]gpsPerfStats, 0) // reinitialize statistics on disconnect / reconnect
		} else {
			mySituation.muGPSPerformance.Lock()
			calculateNavRate()
			mySituation.muGPSPerformance.Unlock()
		}

		for !(globalSettings.IMU_Sensor_Enabled && globalStatus.IMUConnected) && globalStatus.GPS_connected {
			<-timer.C

			if !isGPSValid() || !calcGPSAttitude() {
				if globalSettings.DEBUG {
					log.Printf("Couldn't calculate GPS-based attitude statistics\n")
				}
			} else {
				mySituation.muGPSPerformance.Lock()
				index := len(myGPSPerfStats) - 1
				if index > 1 {
					mySituation.AHRSPitch = myGPSPerfStats[index].gpsPitch
					mySituation.AHRSRoll = myGPSPerfStats[index].gpsRoll
					mySituation.AHRSGyroHeading = float64(mySituation.GPSTrueCourse)
					mySituation.AHRSLastAttitudeTime = stratuxClock.Time

					makeAHRSGDL90Report()
					makeAHRSSimReport()
					makeAHRSLevilReport()
				}
				mySituation.muGPSPerformance.Unlock()
			}
		}
	}
}

func makeAHRSSimReport() {
	msg := createXPlaneAttitudeMsg(float32(mySituation.AHRSGyroHeading), float32(mySituation.AHRSPitch), float32(mySituation.AHRSRoll))
	sendXPlane(msg, 100*time.Millisecond, 1)
}

func makeAHRSGDL90Report() {
	msg := make([]byte, 24)
	msg[0] = 0x4c
	msg[1] = 0x45
	msg[2] = 0x01
	msg[3] = 0x01

	// Values if invalid
	pitch := int16(0x7FFF)
	roll := int16(0x7FFF)
	hdg := int16(0x7FFF)
	slip_skid := int16(0x7FFF)
	yaw_rate := int16(0x7FFF)
	g := int16(0x7FFF)
	airspeed := int16(0x7FFF) // Can add this once we can read airspeed
	palt := uint16(0xFFFF)
	vs := int16(0x7FFF)
	if isAHRSValid() {
		if !isAHRSInvalidValue(mySituation.AHRSPitch) {
			pitch = common.RoundToInt16(mySituation.AHRSPitch * 10)
		}
		if !isAHRSInvalidValue(mySituation.AHRSRoll) {
			roll = common.RoundToInt16(mySituation.AHRSRoll * 10)
		}
		if !isAHRSInvalidValue(mySituation.AHRSGyroHeading) {
			hdg = common.RoundToInt16(mySituation.AHRSGyroHeading * 10)
		}
		if !isAHRSInvalidValue(mySituation.AHRSSlipSkid) {
			slip_skid = common.RoundToInt16(-mySituation.AHRSSlipSkid * 10)
		}
		if !isAHRSInvalidValue(mySituation.AHRSTurnRate) {
			yaw_rate = common.RoundToInt16(mySituation.AHRSTurnRate * 10)
		}
		if !isAHRSInvalidValue(mySituation.AHRSGLoad) {
			g = common.RoundToInt16(mySituation.AHRSGLoad * 10)
		}
	}
	if isTempPressValid() {
		palt = uint16(mySituation.BaroPressureAltitude + 5000.5)
		vs = common.RoundToInt16(float64(mySituation.BaroVerticalSpeed))
	}

	// Roll.
	msg[4] = byte((roll >> 8) & 0xFF)
	msg[5] = byte(roll & 0xFF)

	// Pitch.
	msg[6] = byte((pitch >> 8) & 0xFF)
	msg[7] = byte(pitch & 0xFF)

	// Heading.
	msg[8] = byte((hdg >> 8) & 0xFF)
	msg[9] = byte(hdg & 0xFF)

	// Slip/skid.
	msg[10] = byte((slip_skid >> 8) & 0xFF)
	msg[11] = byte(slip_skid & 0xFF)

	// Yaw rate.
	msg[12] = byte((yaw_rate >> 8) & 0xFF)
	msg[13] = byte(yaw_rate & 0xFF)

	// "G".
	msg[14] = byte((g >> 8) & 0xFF)
	msg[15] = byte(g & 0xFF)

	// Indicated Airspeed
	msg[16] = byte((airspeed >> 8) & 0xFF)
	msg[17] = byte(airspeed & 0xFF)

	// Pressure Altitude
	msg[18] = byte((palt >> 8) & 0xFF)
	msg[19] = byte(palt & 0xFF)

	// Vertical Speed
		msg[20] = byte((vs >> 8) & 0xFF)
	msg[21] = byte(vs & 0xFF)

	// Reserved
	msg[22] = 0x7F
	msg[23] = 0xFF

	sendMsg(prepareMessage(msg), NETWORK_AHRS_GDL90, 100 * time.Millisecond, 3)
}