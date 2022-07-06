package common

import (
	"log"
	"time"
)

type GpsNmeaLineInt interface {
	Print()
}

type GpsNmeaLine struct {
	Name string                       // Unique name, for example SoftRF, uBlox9, or serial port name.. Used for display/logging
	NmeaLine string					  // Received NMEA Line
	GpsTimeOffsetPpsMs time.Duration  // Estimated GPS offset
	GpsDetectedType uint            // This is the type, like OGN, ubloc or SOFTRF
	GpsSource uint                   // This is the source, network, Blue Tooth or serial
}

func (line GpsNmeaLine) Print() {
	log.Printf("Name:%s NMEA:%s time:%d type:%d source:%d\r\n", 
		line.Name,
		line.NmeaLine,
		line.GpsTimeOffsetPpsMs,
		line.GpsDetectedType,
		line.GpsSource,
	)
}