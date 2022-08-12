package gps

import (
	"sync"
	"time"
)

var once sync.Once

const (
	CONTENT_TX_CHANNEL   = 0b00000001
	CONTENT_CONNECTED    = 0b00000010
	CONTENT_TYPE         = 0b00000100
	CONTENT_SOURCE       = 0b00001000
	CONTENT_OFFSET_PPS   = 0b00010000
)

/**
 Structure that is used to announce discovered devices
 */
 type DiscoveredDevice struct {
	Name      string				  // Mandatory: Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	content   uint
	TXChannel chan []byte			  // TX Channel can be used to send a (NMEA) message to a channel
	Connected bool				      // Is the device connected?
	GpsDetectedType uint              // This is the type, like OGN, ublocx or SOFTRF
	GpsSource uint16				  // This is the source, network, Blue Tooth or serial
	GpsTimeOffsetPPS time.Duration    // Estimated GPS offset
	LastDiscoveryMessage  time.Time	  // Last time we heard a discovery message
}

func (d DiscoveredDevice) CanTX() bool {
	return d.content & CONTENT_TX_CHANNEL != 0x00
}

type serviceDiscovery struct {
	C chan DiscoveredDevice  // Channel used to send information about discovered devices
}

var (
	instance serviceDiscovery
)

func GetServiceDiscovery() *serviceDiscovery {

	once.Do(func() { // <-- atomic, does not allow repeating
		instance = serviceDiscovery{
			C: make(chan DiscoveredDevice, 20),
		}
	})

	return &instance
}

func (s *serviceDiscovery) TypeUpgrade(name string, deviceType uint) {
	s.C <- DiscoveredDevice{
		Name: name,
		content: CONTENT_TYPE,
		GpsDetectedType: deviceType,
	}
}

func (s *serviceDiscovery) Connected(name string, c bool) {
	s.C <- DiscoveredDevice{
		Name: name,
		content: CONTENT_CONNECTED,
		Connected: c,
	}
}

func (s *serviceDiscovery) Merge(previous, new DiscoveredDevice) DiscoveredDevice {
	if new.content & CONTENT_TX_CHANNEL == 0x00 {
		new.TXChannel = previous.TXChannel
	}
	if new.content & CONTENT_CONNECTED == 0x00 {
		new.Connected = previous.Connected
	}
	if new.content & CONTENT_SOURCE == 0x00 {
		new.GpsSource = previous.GpsSource
	}
	if new.content & CONTENT_OFFSET_PPS == 0x00 {
		new.GpsTimeOffsetPPS = previous.GpsTimeOffsetPPS
	}
	if new.content & CONTENT_TYPE == 0x00 {
		new.GpsDetectedType = previous.GpsDetectedType
	}

	new.LastDiscoveryMessage = previous.LastDiscoveryMessage
	new.content = previous.content | new.content

	return new
}

func (s *serviceDiscovery) Send(d DiscoveredDevice) {
	s.C <- d
}


