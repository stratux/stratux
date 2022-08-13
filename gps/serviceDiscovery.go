package gps

import (
	"sync"
	"time"
)



const (
	CONTENT_TX_CHANNEL   = 0b00000001	// When set, DiscoveredDevice has a TX channel
	CONTENT_CONNECTED    = 0b00000010	// When set, DiscoveredDevice jas a valid connected state
	CONTENT_TYPE         = 0b00000100	// When set, DiscoveredDevice has a valid GPS Source type
	CONTENT_SOURCE       = 0b00001000	// When set, DiscoveredDevice has a valid GPS Source
	CONTENT_OFFSET_PPS   = 0b00010000	// When set, DiscoveredDevice has a valid GPS Time Offset PPS
)	

/**
 Structure that is used to announce discovered devices
 */
 type DiscoveredDevice struct {
	Name      string				  // Mandatory: Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	content   uint
	TXChannel chan []byte			  // TX Channel can be used to send a (NMEA) message to a channel
	Connected bool				      // Is the device connected?
	GPSDetectedType uint              // This is the type, like OGN, ublocx or SOFTRF
	GPSSource uint16				  // This is the source, network, Blue Tooth or serial
	GPSTimeOffsetPPS time.Duration    // Time it takes for a message with time to arrive in stratux
	LastDiscoveryMessage  time.Time	  // Last time we heard a discovery message
}

type serviceDiscovery struct {
	C chan DiscoveredDevice  // Channel used to send information about discovered devices
}

var (
	onceDiscoveredDevice sync.Once
	serviceDiscoveryInstance serviceDiscovery
)

func (d DiscoveredDevice) CanTX() bool {
	return d.content & CONTENT_TX_CHANNEL != 0x00
}


func GetServiceDiscovery() *serviceDiscovery {

	onceDiscoveredDevice.Do(func() { 
		serviceDiscoveryInstance = serviceDiscovery{
			C: make(chan DiscoveredDevice, 20),
		}
	})

	return &serviceDiscoveryInstance
}

func (s *serviceDiscovery) TypeUpgrade(name string, deviceType uint) {
	s.C <- DiscoveredDevice{
		Name: name,
		content: CONTENT_TYPE,
		GPSDetectedType: deviceType,
	}
}

func (s *serviceDiscovery) Connected(name string, c bool) {
	s.C <- DiscoveredDevice{
		Name: name,
		content: CONTENT_CONNECTED,
		Connected: c,
	}
}

func (s *serviceDiscovery) Send(d DiscoveredDevice) {
	s.C <- d
}

// Merge the previous and current DiscoveredDevice based on it's content
func (s *serviceDiscovery) Merge(previous, new DiscoveredDevice) DiscoveredDevice {
	if new.content & CONTENT_TX_CHANNEL == 0x00 {
		new.TXChannel = previous.TXChannel
	}
	if new.content & CONTENT_CONNECTED == 0x00 {
		new.Connected = previous.Connected
	}
	if new.content & CONTENT_SOURCE == 0x00 {
		new.GPSSource = previous.GPSSource
	}
	if new.content & CONTENT_OFFSET_PPS == 0x00 {
		new.GPSTimeOffsetPPS = previous.GPSTimeOffsetPPS
	}
	if new.content & CONTENT_TYPE == 0x00 {
		new.GPSDetectedType = previous.GPSDetectedType
	}

	new.LastDiscoveryMessage = previous.LastDiscoveryMessage
	new.content = previous.content | new.content

	return new
}
