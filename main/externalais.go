/*
	Copyright (c) 2024
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	externalais.go: Routines for reading AIS traffic from external serial receivers
	                (e.g., Daisy 2+, Daisy HAT, or other NMEA AIS receivers)
*/

package main

import (
	"bufio"
	"log"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/stratux/serial"
)

var externalAISSerialConfig *serial.Config
var externalAISSerialPort *serial.Port
var externalAISExitChan chan bool = make(chan bool, 1)

// externalAISListen is the main goroutine for reading AIS data from an external serial receiver
func externalAISListen() {
	for {
		if !globalSettings.ExternalAIS_Enabled {
			// Wait until external AIS is enabled
			time.Sleep(1 * time.Second)
			continue
		}

		// Try to connect to the external AIS device
		if !initExternalAISSerial() {
			time.Sleep(3 * time.Second)
			continue
		}

		log.Printf("External AIS: successfully connected to %s at %d baud\n",
			globalSettings.ExternalAIS_SerialPort, globalSettings.ExternalAIS_BaudRate)
		globalStatus.ExternalAIS_connected = true

		// Make sure the exit channel is empty
		for len(externalAISExitChan) > 0 {
			<-externalAISExitChan
		}

		// Read loop
		reader := bufio.NewReader(externalAISSerialPort)
		for globalSettings.ExternalAIS_Enabled {
			// Set read timeout
			externalAISSerialPort.SetReadDeadline(time.Now().Add(5 * time.Second))

			line, err := reader.ReadString('\n')
			if err != nil {
				if strings.Contains(err.Error(), "timeout") {
					// Timeout is expected, just continue
					continue
				}
				log.Printf("External AIS: read error: %s\n", err.Error())
				break
			}

			line = strings.TrimSpace(line)
			if len(line) == 0 {
				continue
			}

			// Process AIS NMEA sentences (!AIVDM, !AIVDO)
			if strings.HasPrefix(line, "!AIVDM") || strings.HasPrefix(line, "!AIVDO") {
				TraceLog.Record(CONTEXT_AIS, []byte(line))
				parseAisMessage(line)
			}
		}

		// Cleanup
		globalStatus.ExternalAIS_connected = false
		if externalAISSerialPort != nil {
			externalAISSerialPort.Close()
			externalAISSerialPort = nil
		}
		time.Sleep(3 * time.Second)
	}
}

// initExternalAISSerial initializes the serial port for the external AIS receiver
func initExternalAISSerial() bool {
	device := globalSettings.ExternalAIS_SerialPort
	baudrate := globalSettings.ExternalAIS_BaudRate

	// Validate baud rate
	if baudrate <= 0 {
		baudrate = 38400 // Default baud rate for most AIS receivers
	}

	// Check if device exists
	if _, err := os.Stat(device); err != nil {
		if globalSettings.DEBUG {
			log.Printf("External AIS: device %s not found\n", device)
		}
		return false
	}

	// Open serial port
	externalAISSerialConfig = &serial.Config{
		Name:        device,
		Baud:        baudrate,
		ReadTimeout: 5 * time.Second,
	}

	p, err := serial.OpenPort(externalAISSerialConfig)
	if err != nil {
		log.Printf("External AIS: error opening serial port %s: %s\n", device, err.Error())
		return false
	}

	externalAISSerialPort = p
	return true
}

// closeExternalAISSerial closes the serial port
func closeExternalAISSerial() {
	if externalAISSerialPort != nil {
		externalAISSerialPort.Close()
		externalAISSerialPort = nil
	}
	globalStatus.ExternalAIS_connected = false
}

// GetAvailableSerialPorts returns a list of available serial ports for the web interface
func GetAvailableSerialPorts() []string {
	var ports []string

	// Common serial port patterns
	patterns := []string{
		"/dev/ttyUSB*",
		"/dev/ttyACM*",
		"/dev/ttyAMA*",
		"/dev/serial*",
	}

	for _, pattern := range patterns {
		matches, err := filepath.Glob(pattern)
		if err == nil {
			ports = append(ports, matches...)
		}
	}

	// Add common defaults if they exist
	defaults := []string{"/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyAMA0"}
	for _, d := range defaults {
		found := false
		for _, p := range ports {
			if p == d {
				found = true
				break
			}
		}
		if !found {
			if _, err := os.Stat(d); err == nil {
				ports = append(ports, d)
			}
		}
	}

	return ports
}
