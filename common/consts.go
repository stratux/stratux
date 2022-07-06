package common

const (
	/*
		GPS_TYPE_NMEA     = 0x01
		GPS_TYPE_UBX      = 0x02
		GPS_TYPE_SIRF     = 0x03
		GPS_TYPE_MEDIATEK = 0x04
		GPS_TYPE_FLARM    = 0x05
		GPS_TYPE_GARMIN   = 0x06
	*/

	GPS_TYPE_NONE          = 0x00
	GPS_TYPE_UBX9          = 0x09
	GPS_TYPE_UBX8          = 0x08
	GPS_TYPE_UBX7          = 0x07
	GPS_TYPE_UBX6          = 0x06
	GPS_TYPE_PROLIFIC      = 0x02
	GPS_TYPE_UART          = 0x01
	GPS_TYPE_SERIAL        = 0x0A
	GPS_TYPE_OGNTRACKER    = 0x03
	GPS_TYPE_SOFTRF_DONGLE = 0x0B
	GPS_TYPE_NETWORK       = 0x0C
	GPS_PROTOCOL_NMEA      = 0x10
	// other GPS types to be defined as needed
)

// When we have multiple GPS connected, we use this to show our preference if one or more
// GPSes do have an fix
const (
	GPS_PREFER_SERIAL		= 0x01
	GPS_PREFER_BLUETOOTH	= 0x02
	GPS_PREFER_NETWORK	    = 0x03
	GPS_PREFER_BEST_FIX	    = 0x04 
)

const (
	GPS_SOURCE_UNKNOWN		= 0x00
	GPS_SOURCE_NETWORK		= 0x01
	GPS_SOURCE_SERIAL   	= 0x02
	GPS_SOURCE_BLE	        = 0x03
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
