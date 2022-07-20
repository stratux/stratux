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
}

/** 
Structure to be use to send a message to a GPS device, for example to configure it
*/
type TXMessage struct {
	Message []byte				      // Message to send
	Name string                       // Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
}

/**
 Structure that is used to announce discovered devices
 */
type DiscoveredDevice struct {
	Name      string				  // Mandatory: Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	Connected bool					  // Mandatory: We true we have an actualy connected to the device and can receive/send messages
	LastDiscoveryMessage  uint64				  // Last time we heard a discovery message
	GpsDetectedType uint              // mandatory: This is the type, like OGN, ublocx or SOFTRF
	GpsSource uint16					  // mandatory: This is the source, network, Blue Tooth or serial
	HasTXChannel bool				  // mandatory: True when this message contains a valid TXChannel
	TXChannel chan []byte			  // TX Channel can be used to send a (NMEA) message to a channel
    GpsTimeOffsetPpsMs time.Duration  // mandatory: Estimated GPS offset
}

type DiscoveredDeviceDTO struct {
	Name      string				  // Name of the device
	Connected bool					  // We true we have an actualy connected to the device and can receive/send messages
	LastDiscoveryMessage  uint64                  // Last time we heard from this device
	GpsDetectedType uint              // This is the type, like OGN, ubloc or SOFTRF
	GpsSource uint16					  // This is the source, network, Blue Tooth or serial
}

func (line *RXMessage) Print() {
	log.Printf("Name:%s NMEA:%s\r\n", 
		line.Name,
		line.NmeaLine,
	)
}

func (line *DiscoveredDevice) Print() {
	log.Printf("Name:%s Connected:%t hasTXChannel:%t time:%d type:%d source:%d\r\n", 
		line.Name,
		line.Connected,
		line.HasTXChannel,
		line.LastDiscoveryMessage,
		line.GpsDetectedType,
		line.GpsSource,
	)
}
