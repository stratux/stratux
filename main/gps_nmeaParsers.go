package main

import (
	"errors"
	"fmt"
	"strconv"
	"time"

	"github.com/b3nn0/stratux/v2/common"
)

var EMPTY_SITUATION  = SituationData{}

func calculateNACp(accuracy float32) uint8 {
	ret := uint8(0)

	if accuracy < 3 {
		ret = 11
	} else if accuracy < 10 {
		ret = 10
	} else if accuracy < 30 {
		ret = 9
	} else if accuracy < 92.6 {
		ret = 8
	} else if accuracy < 185.2 {
		ret = 7
	} else if accuracy < 555.6 {
		ret = 6
	}

	return ret
}

func getsvTypesvStr(s string) (sv int, svType uint8, svStr string, err error) {
	sv, err = strconv.Atoi(s) // sv number
	if err != nil {
		return 0, 0, "", errors.New("Invalid svType")
	}

	if sv <= 32 {
		svType = common.SAT_TYPE_GPS
		svStr = fmt.Sprintf("G%d", sv) // GPS 1-32
	} else if sv <= 64 {
		svType = common.SAT_TYPE_SBAS
		svStr = fmt.Sprintf("S%d", sv+87) // SBAS 33-64, 33 = SBAS PRN 120
	} else if sv <= 96 {
		svType = common.SAT_TYPE_GLONASS
		svStr = fmt.Sprintf("R%d", sv-64) // GLONASS 65-96
	} else if sv <= 158 {
		svType = common.SAT_TYPE_SBAS
		svStr = fmt.Sprintf("S%d", sv-151) // SBAS 152-158
	} else if sv <= 202 {
		svType = common.SAT_TYPE_QZSS
		svStr = fmt.Sprintf("Q%d", sv-192) // QZSS 193-202
	} else if sv <= 336 {
		svType = common.SAT_TYPE_GALILEO
		svStr = fmt.Sprintf("E%d", sv-300) // GALILEO 301-336
	} else if sv <= 437 {
		svType = common.SAT_TYPE_BEIDOU
		svStr = fmt.Sprintf("B%d", sv-400) // BEIDOU 401-437
	} else {
		svType = common.SAT_TYPE_UNKNOWN
		svStr = fmt.Sprintf("U%d", sv)
	}

	return
}

/** Process GNVTG & GPVTG into a SituationData
  function will not have side effects!x
*/
func parseNMEALine_GNVTG_GPVTG(x []string, tmpSituation *SituationData) (SituationData, error) {
	if (!(x[0] == "GNVTG") || (x[0] == "GPVTG")) {
		return *tmpSituation, errors.New("Not GNVTG GPVTG")
	}

	if len(x) < 9 { // Reduce from 10 to 9 to allow parsing by devices pre-NMEA v2.3
		return EMPTY_SITUATION, errors.New("Length < 9")
	}

	groundspeed, err := strconv.ParseFloat(x[5], 32) // Knots.
	if err != nil {
		return EMPTY_SITUATION, errors.New("Ground speed not found")
	}
	tmpSituation.GPSGroundSpeed = groundspeed

	trueCourse := float32(0)
	tc, err := strconv.ParseFloat(x[1], 32)
	if err != nil {
		return EMPTY_SITUATION, errors.New("True Course not found")
	}
	if groundspeed > 3 { //TODO: use average groundspeed over last n seconds to avoid random "jumps"
		trueCourse = float32(tc)
		// Not used setTrueCourse(uint16(groundspeed), tc)
		tmpSituation.GPSTrueCourse = trueCourse
	} else {
		// Negligible movement. Don't update course, but do use the slow speed.
		//TODO: use average course over last n seconds?
	}
	tmpSituation.GPSLastGroundTrackTime = stratuxClock.Time

	// We've made it this far, so that means we've processed "everything" and can now make the change to mySituation.
	return *tmpSituation, nil
}

func parseNMEALine_GNGGA_GPGGA(x []string, tmpSituation *SituationData) (SituationData, error) {
	if (!(x[0] == "GNGGA") || (x[0] == "GPGGA")) {
		return *tmpSituation, errors.New("Not GNGGA GPGGA")
	}
	
	if len(x) < 15 {
		return EMPTY_SITUATION, errors.New("Length < 15")
	}

	// GPSFixQuality indicator.
	q, err1 := strconv.Atoi(x[6])
	if err1 != nil {
		return EMPTY_SITUATION, errors.New("GPSFixQuality not found")
	}
	tmpSituation.GPSFixQuality = uint8(q) // 1 = 3D GPS; 2 = DGPS (SBAS /WAAS)

	// Timestamp.
	if len(x[1]) < 7 {
		return EMPTY_SITUATION, errors.New("Timestamp not found")
	}
	hr, err1 := strconv.Atoi(x[1][0:2])
	min, err2 := strconv.Atoi(x[1][2:4])
	sec, err3 := strconv.ParseFloat(x[1][4:], 32)
	if err1 != nil || err2 != nil || err3 != nil {
		return EMPTY_SITUATION, errors.New("Timestamp not found")
	}

	tmpSituation.GPSLastFixSinceMidnightUTC = float32(3600*hr+60*min) + float32(sec)

	// Latitude.
	if len(x[2]) < 4 {
		return EMPTY_SITUATION, errors.New("Latitude not found")
	}

	hr, err1 = strconv.Atoi(x[2][0:2])
	minf, err2 := strconv.ParseFloat(x[2][2:], 32)
	if err1 != nil || err2 != nil {
		return EMPTY_SITUATION, errors.New("Latitude format incorrect")
	}

	tmpSituation.GPSLatitude = float32(hr) + float32(minf/60.0)
	if x[3] == "S" { // South = negative.
		tmpSituation.GPSLatitude = -tmpSituation.GPSLatitude
	}

	// Longitude.
	if len(x[4]) < 5 {
		return EMPTY_SITUATION, errors.New("Longitude not found")
	}
	hr, err1 = strconv.Atoi(x[4][0:3])
	minf, err2 = strconv.ParseFloat(x[4][3:], 32)
	if err1 != nil || err2 != nil {
		return EMPTY_SITUATION, errors.New("Longitude format incorrect")
	}

	tmpSituation.GPSLongitude = float32(hr) + float32(minf/60.0)
	if x[5] == "W" { // West = negative.
		tmpSituation.GPSLongitude = -tmpSituation.GPSLongitude
	}

	// Altitude.
	alt, err1 := strconv.ParseFloat(x[9], 32)
	if err1 != nil {
		return EMPTY_SITUATION, errors.New("Altitude not found")
	}
	tmpSituation.GPSAltitudeMSL = float32(alt * 3.28084) // Convert to feet.

	// Geoid separation (Sep = HAE - MSL)
	geoidSep, err1 := strconv.ParseFloat(x[11], 32)
	if err1 != nil {
		return EMPTY_SITUATION, errors.New("Geoid separation not found")
	}
	tmpSituation.GPSGeoidSep = float32(geoidSep * 3.28084) // Convert to feet.
	tmpSituation.GPSHeightAboveEllipsoid = tmpSituation.GPSGeoidSep + tmpSituation.GPSAltitudeMSL

	// Timestamp.
	tmpSituation.GPSLastFixLocalTime = stratuxClock.Time

	return *tmpSituation, nil
}

func parseNMEALine_GNRMC_GPRMC(x []string, tmpSituation *SituationData) (SituationData, error) {
	if (!(x[0] == "GNRMC") || (x[0] == "GPRMC")) {
		return *tmpSituation, errors.New("Not GNRMC GPRMC")
	}

	//$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
	/*						check RY835 man for NMEA version, if >2.2, add mode field
		Where:
	 RMC          Recommended Minimum sentence C
	 123519       Fix taken at 12:35:19 UTC
	 A            Status A=active or V=Void.
	 4807.038,N   Latitude 48 deg 07.038' N
	 01131.000,E  Longitude 11 deg 31.000' E
	 022.4        Speed over the ground in knots
	 084.4        Track angle in degrees True
	 230394       Date - 23rd of March 1994
	 003.1,W      Magnetic Variation
	 D			  mode field (nmea 2.3 and higher)
	 *6A          The checksum data, always begins with *
	*/
	if len(x) < 11 {
		return EMPTY_SITUATION, errors.New("Length < 11")
	}

	if x[2] != "A" { // invalid fix
		tmpSituation.GPSFixQuality = 0 // Just a note.
		return *tmpSituation, nil
	}

	// Timestamp. (note: GPZDA is not send by some devices)
	if len(x[1]) < 7 {
		return EMPTY_SITUATION, errors.New("Timestamp not found")
	}
	hr, err1 := strconv.Atoi(x[1][0:2])
	min, err2 := strconv.Atoi(x[1][2:4])
	sec, err3 := strconv.ParseFloat(x[1][4:], 32)
	if err1 != nil || err2 != nil || err3 != nil {
		return EMPTY_SITUATION, errors.New("Timestamp format incorrect")
	}
	tmpSituation.GPSLastFixSinceMidnightUTC = float32(3600*hr+60*min) + float32(sec)

	if len(x[9]) == 6 {
		// Date of Fix, i.e 191115 =  19 November 2015 UTC  field 9
		gpsTimeStr := fmt.Sprintf("%s %02d:%02d:%06.3f", x[9], hr, min, sec)
		gpsTime, err := time.Parse("020106 15:04:05.000", gpsTimeStr)
		gpsTime = gpsTime.Add(gpsTimeOffsetPpsMs)                                                // rough estimate for PPS offset
		if err == nil && gpsTime.After(time.Date(2016, time.January, 0, 0, 0, 0, 0, time.UTC)) { // Ignore dates before 2016-JAN-01.
			tmpSituation.GPSLastGPSTimeStratuxTime = stratuxClock.Time
			tmpSituation.GPSTime = gpsTime
		}
	}

	// Latitude.
	if len(x[3]) < 4 {
		return EMPTY_SITUATION, errors.New("Latitude not found")
	}
	hr, err1 = strconv.Atoi(x[3][0:2])
	minf, err2 := strconv.ParseFloat(x[3][2:], 32)
	if err1 != nil || err2 != nil {
		return EMPTY_SITUATION, errors.New("Latitude format incorrect")
	}
	tmpSituation.GPSLatitude = float32(hr) + float32(minf/60.0)
	if x[4] == "S" { // South = negative.
		tmpSituation.GPSLatitude = -tmpSituation.GPSLatitude
	}
	// Longitude.
	if len(x[5]) < 5 {
		return EMPTY_SITUATION, errors.New("Longitude not found")
	}
	hr, err1 = strconv.Atoi(x[5][0:3])
	minf, err2 = strconv.ParseFloat(x[5][3:], 32)
	if err1 != nil || err2 != nil {
		return EMPTY_SITUATION, errors.New("Longitude format incorrect")
	}
	tmpSituation.GPSLongitude = float32(hr) + float32(minf/60.0)
	if x[6] == "W" { // West = negative.
		tmpSituation.GPSLongitude = -tmpSituation.GPSLongitude
	}

	tmpSituation.GPSLastFixLocalTime = stratuxClock.Time

	// ground speed in kts (field 7)
	groundspeed, err := strconv.ParseFloat(x[7], 32)
	if err != nil {
		return EMPTY_SITUATION, errors.New("Groundspeed not found")
	}
	tmpSituation.GPSGroundSpeed = groundspeed

	// ground track "True" (field 8)
	tc, err := strconv.ParseFloat(x[8], 32)
	if err != nil && groundspeed > 3 { // some receivers return null COG at low speeds. Need to ignore this condition.
		return EMPTY_SITUATION, errors.New("Truecourse not found")
	}
	tmpSituation.GPSTrueCourse = float32(tc)
	tmpSituation.GPSLastGroundTrackTime = stratuxClock.Time

	return *tmpSituation, nil
}

func parseNMEALine_GNGSA_GPGSA_GLGSA_GAGSA_GBGSA(x []string, tmpSituation *SituationData) (SituationData, error) {
	if (!(x[0] == "GNGSA") || (x[0] == "GPGSA") || (x[0] == "GLGSA") || (x[0] == "GAGSA") || (x[0] == "GBGSA")) {
		return *tmpSituation, errors.New("Not GNGSA GPGSA GLGSA GAGSA GBGSA")
	}

	if len(x) < 18 {
		return EMPTY_SITUATION, errors.New("Length < 18")
	}

	// field 1: operation mode
	// M: manual forced to 2D or 3D mode
	// A: automatic switching between 2D and 3D modes

	/*
		if (x[1] != "A") && (x[1] != "M") { // invalid fix ... but x[2] is a better indicator of fix quality. Deprecating this.
			tmpSituation.GPSFixQuality = 0 // Just a note.
			return false
		}
	*/

	// field 2: solution type
	// 1 = no solution; 2 = 2D fix, 3 = 3D fix. WAAS status is parsed from GGA message, so no need to get here
	if (x[2] == "") || (x[2] == "1") { // missing or no solution
		tmpSituation.GPSFixQuality = 0 // Just a note.
		return *tmpSituation, nil
	}

	// field 16: HDOP
	// Accuracy estimate
	hdop, err1 := strconv.ParseFloat(x[16], 32)
	if err1 != nil {
		return EMPTY_SITUATION, errors.New("HDOP not found")
	}
	// We store hdop in GPSHorizontalAccuracy, but we need to process that latr with the multiplication factor
	// This is due to the factor depends on the GPS type (ublox etc..)
	tmpSituation.GPSHorizontalAccuracy = float32(hdop)

	// NACp estimate.
	tmpSituation.GPSNACp = calculateNACp(tmpSituation.GPSHorizontalAccuracy)

	// field 17: VDOP
	// accuracy estimate
	vdop, err1 := strconv.ParseFloat(x[17], 32)
	if err1 != nil {
		return EMPTY_SITUATION, errors.New("VDOP not found")
	}
	tmpSituation.GPSVerticalAccuracy = float32(vdop * 5) // rough estimate for 95% confidence

	return *tmpSituation, nil
}

// note: parseNMEALine_GPGSV_GLGSV_GAGSV_GBGSV is only validating the first part of the message
func parseNMEALine_GPGSV_GLGSV_GAGSV_GBGSV(x []string, tmpSituation *SituationData) (SituationData, error) {
	if (!(x[0] == "GPGSV") || (x[0] == "GLGSV") || (x[0] == "GAGSV") || (x[0] == "GBGSV")) {
		return *tmpSituation, errors.New("Not GPGSV GLGSV GAGSV GBGSV")
	}
	
	if len(x) < 4 {
		return EMPTY_SITUATION, errors.New("Length < 4")
	}

	// field 1 = number of GSV messages of this type
	_, err := strconv.Atoi(x[2]) // TODO: RVT is this not x[1] ??
	if err != nil {
		return EMPTY_SITUATION, errors.New("GSV field not found")
	}

	// field 2 = index of this GSV message

	_, err = strconv.Atoi(x[2])
	if err != nil {
		return EMPTY_SITUATION, errors.New("GSV field not found")
	}

	return *tmpSituation, nil
}

func parseNMEALine_POGNB(x []string, tmpSituation *SituationData) (SituationData, error) {
	if (!(x[0] == "POGNB")) {
		return *tmpSituation, errors.New("Not POGNB")
	}

	// OGN Tracker pressure data:
	// $POGNB,22.0,+29.1,100972.3,3.8,+29.4,+87.2,-0.04,+32.6,*6B
	if len(x) < 5 {
		return EMPTY_SITUATION, errors.New("Length < 5")
	}
	var vspeed float64

	pressureAlt, err := strconv.ParseFloat(x[5], 32)
	if err != nil {
		return EMPTY_SITUATION, errors.New("Pressure Altitude not found")
	}

	vspeed, err = strconv.ParseFloat(x[7], 32)
	if err != nil {
		return EMPTY_SITUATION, errors.New("Vertical Speed not found")
	}

	if !isTempPressValid2(*tmpSituation) || tmpSituation.BaroSourceType != common.BARO_TYPE_BMP280 {
		tmpSituation.BaroPressureAltitude = float32(pressureAlt * 3.28084) // meters to feet
		tmpSituation.BaroVerticalSpeed = float32(vspeed * 196.85)          // m/s in ft/min
		tmpSituation.BaroLastMeasurementTime = stratuxClock.Time
		tmpSituation.BaroSourceType = common.BARO_TYPE_OGNTRACKER
	}

	return *tmpSituation, nil
}
