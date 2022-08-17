/*
	Copyright (c) 2022 R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.
	aprs.go: Routines for reading traffic from aprs
*/
package gps

import ()

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
	GPS_TYPE_SOFTRF_AT65   = 0x0E
	GPS_TYPE_NETWORK       = 0x0C
	GPS_PROTOCOL_NMEA      = 0x10 // TODO: RVT: This should really be a seperate variable in globalStatus, is this used externall outside of Stratux?
	GPS_TYPE_BLUETOOTH     = 0x0D
	// other GPS types to be defined as needed
)

const (
	GPS_SOURCE_SERIAL    = 0x01
	GPS_SOURCE_BLUETOOTH = 0x02
	GPS_SOURCE_NETWORK   = 0x03
)

/**
Structure for message that will be send from GPS devices
*/
type RXMessage struct {
	Name     string // Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	NmeaLine string // Received NMEA Line
}

/**
Structure to be use to send a message to a GPS device, for example to configure it
*/
type TXMessage struct {
	Message []byte // Message to send
	Name    string // Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
}

type DiscoveredDeviceDTO struct {
	Name            string // Name of the device
	MAC             string // MAC address of the device
	Connected       bool   // We true we have an actualy connected to the device and can receive/send messages
	GPSDetectedType uint   // This is the type, like OGN, ubloc or SOFTRF
	GPSSource       uint16 // This is the source, network, Blue Tooth or serial
}
