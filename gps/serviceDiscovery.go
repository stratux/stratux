/*
	Copyright (c) 2022 R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.
	aprs.go: Routines for reading traffic from aprs
*/
package gps

import (
	"sync"
	"time"
)

const (
	CONTENT_TX_CHANNEL = 0b00000001 // When set, DiscoveredDevice has a TX channel
	CONTENT_CONNECTED  = 0b00000010 // When set, DiscoveredDevice jas a valid connected state
	CONTENT_TYPE       = 0b00000100 // When set, DiscoveredDevice has a valid GPS Source type
	CONTENT_SOURCE     = 0b00001000 // When set, DiscoveredDevice has a valid GPS Source
	CONTENT_OFFSET_PPS = 0b00010000 // When set, DiscoveredDevice has a valid GPS Time Offset PPS
	CONTENT_TIME_GNGGA = 0b00100000 // When set we will set time only from GXGGA messages
)

/**
Structure that is used to announce discovered devices
*/
type DiscoveredDevice struct {
	Name               string        // Mandatory: Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	Content            uint          
	MAC                string        // Optional: MAC address of the device
	TXChannel          chan []byte   // TX Channel can be used to send a (NMEA) message to a channel
	Connected          bool          // Is the device connected?
	GPSDetectedType    uint          // This is the type, like OGN, ublocx or SOFTRF
	GPSSource          uint16        // This is the source, network, Blue Tooth or serial
	GPSTimeOffsetPPS   time.Duration // Time it takes for a message with time to arrive in stratux
	SetTimeFromGGAOnly bool          // Set the time from a GGA message
}

type serviceDiscovery struct {
	C chan DiscoveredDevice // Channel used to send information about discovered devices
}

var (
	onceDiscoveredDevice     sync.Once
	serviceDiscoveryInstance serviceDiscovery
)

func (d DiscoveredDevice) CanTX() bool {
	return d.Content&CONTENT_TX_CHANNEL != 0x00
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
		Name:            name,
		Content:         CONTENT_TYPE,
		GPSDetectedType: deviceType,
	}
}

func (s *serviceDiscovery) TimeFromGXGGAOnly(name string, c bool) {
	s.C <- DiscoveredDevice{
		Name:               name,
		Content:            CONTENT_TIME_GNGGA,
		SetTimeFromGGAOnly: c,
	}
}

func (s *serviceDiscovery) Connected(name string, c bool) {
	s.C <- DiscoveredDevice{
		Name:      name,
		Content:   CONTENT_CONNECTED,
		Connected: c,
	}
}

func (s *serviceDiscovery) Send(d DiscoveredDevice) {
	s.C <- d
}

// Merge the previous and current DiscoveredDevice based on it's content
func (s *serviceDiscovery) Merge(previous, new DiscoveredDevice) DiscoveredDevice {
	if new.Content&CONTENT_TX_CHANNEL == 0x00 {
		new.TXChannel = previous.TXChannel
	}
	if new.Content&CONTENT_CONNECTED == 0x00 {
		new.Connected = previous.Connected
	}
	if new.Content&CONTENT_SOURCE == 0x00 {
		new.GPSSource = previous.GPSSource
	}
	if new.Content&CONTENT_OFFSET_PPS == 0x00 {
		new.GPSTimeOffsetPPS = previous.GPSTimeOffsetPPS
	}
	if new.Content&CONTENT_TYPE == 0x00 {
		new.GPSDetectedType = previous.GPSDetectedType
	}

	if new.Content&CONTENT_TIME_GNGGA == 0x00 {
		new.SetTimeFromGGAOnly = previous.SetTimeFromGGAOnly
	}

	new.Content = previous.Content | new.Content

	return new
}
