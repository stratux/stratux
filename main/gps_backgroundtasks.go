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
	"github.com/b3nn0/stratux/v2/common"
	"log"
	"math"
	"time"
)

var gnssBaroAltDiffs = make(map[int]int)

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

func baroAltGuesser() {
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
		if isGPSValid() && (!isTempPressValid() || mySituation.BaroSourceType == common.BARO_TYPE_NONE || mySituation.BaroSourceType == common.BARO_TYPE_ADSBESTIMATE) {
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
					mySituation.BaroSourceType = common.BARO_TYPE_ADSBESTIMATE
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

func ffAttitudeSender() {
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

func gpsAttitudeSender() {
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

		for !(globalSettings.IMU_Sensor_Enabled && globalStatus.IMUConnected) && (globalSettings.GPS_Enabled && globalStatus.GPS_connected) {
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