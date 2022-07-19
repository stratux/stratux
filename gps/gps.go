package gps

import (
	"log"
	"time"
)

/**
Structure for message that will be send from GPS devices
*/
type RXMessage struct {
	Name string                       // Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	NmeaLine string					  // Received NMEA Line
	GpsTimeOffsetPpsMs time.Duration  // Estimated GPS offset
	GpsDetectedType uint              // This is the type, like OGN, ubloc or SOFTRF
	GpsSource uint                    // This is the source, network, Blue Tooth or serial
}

/** 
Structure to be use to send a message to a GPS device, for example to configure it
*/
type TXMessage struct {
	Message string				      // Message to send
	Name string                       // Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
// TODO: RVT: Not sure yet if we should do this on exact name, or that we are ok with detected type and source, for example any SoftRF connected over bluetooth..
//	GpsDetectedType uint              // This is the type, like OGN, ubloc or SOFTRF
//	GpsSource uint                    // This is the source, network, Blue Tooth or serial
}

/**
 Structure that is used to announce discovered devices
 */
type DiscoveredDevice struct {
	Name      string				  // Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	Connected bool					  // We true we have an actualy connected to the device and can receive/send messages
	LastSeen  uint64				  // Last time we heard from this device
	GpsDetectedType uint              // This is the type, like OGN, ubloc or SOFTRF
	GpsSource uint					  // This is the source, network, Blue Tooth or serial
	TXChannel chan string			  // TX Channel can be used to send a (NMEA) message to a channel
	HasTXChannel bool				  // True when this message contains a valid TXChannel
}

type DiscoveredDeviceDTO struct {
	Name      string				  // Name of the device
	Connected bool					  // We true we have an actualy connected to the device and can receive/send messages
	LastSeen  uint64                  // Last time we heard from this device
	GpsDetectedType uint              // This is the type, like OGN, ubloc or SOFTRF
	GpsSource uint					  // This is the source, network, Blue Tooth or serial
}

func (line *RXMessage) Print() {
	log.Printf("Name:%s NMEA:%s time:%d type:%d source:%d\r\n", 
		line.Name,
		line.NmeaLine,
		line.GpsTimeOffsetPpsMs,
		line.GpsDetectedType,
		line.GpsSource,
	)
}

func (line *DiscoveredDevice) Print() {
	log.Printf("Name:%s Connected:%t hasTXChannel:%t time:%d type:%d source:%d\r\n", 
		line.Name,
		line.Connected,
		line.HasTXChannel,
		line.LastSeen,
		line.GpsDetectedType,
		line.GpsSource,
	)
}
