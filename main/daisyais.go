/*
	Copyright (c) 2024
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	daisyais.go: Routines for reading AIS traffic from Daisy serial receivers
	             (Daisy 2+ and Daisy HAT from Wegmatt)
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

var daisySerialConfig *serial.Config
var daisySerialPort *serial.Port
var daisyExitChan chan bool = make(chan bool, 1)

// daisyAISListen is the main goroutine for reading AIS data from a Daisy receiver
func daisyAISListen() {
	for {
		if !globalSettings.DaisyAIS_Enabled {
			// Wait until Daisy AIS is enabled
			time.Sleep(1 * time.Second)
			continue
		}

		// Try to connect to the Daisy device
		if !initDaisySerial() {
			time.Sleep(3 * time.Second)
			continue
		}

		log.Printf("Daisy AIS: successfully connected to %s at %d baud\n",
			globalSettings.DaisyAIS_SerialPort, globalSettings.DaisyAIS_BaudRate)
		globalStatus.DaisyAIS_connected = true

		// Make sure the exit channel is empty
		for len(daisyExitChan) > 0 {
			<-daisyExitChan
		}

		// Read loop
		reader := bufio.NewReader(daisySerialPort)
		for globalSettings.DaisyAIS_Enabled {
			// Set read timeout
			daisySerialPort.SetReadDeadline(time.Now().Add(5 * time.Second))

			line, err := reader.ReadString('\n')
			if err != nil {
				if strings.Contains(err.Error(), "timeout") {
					// Timeout is expected, just continue
					continue
				}
				log.Printf("Daisy AIS: read error: %s\n", err.Error())
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
		globalStatus.DaisyAIS_connected = false
		if daisySerialPort != nil {
			daisySerialPort.Close()
			daisySerialPort = nil
		}
		time.Sleep(3 * time.Second)
	}
}

// initDaisySerial initializes the serial port for the Daisy receiver
func initDaisySerial() bool {
	device := globalSettings.DaisyAIS_SerialPort
	baudrate := globalSettings.DaisyAIS_BaudRate

	// Validate baud rate
	if baudrate <= 0 {
		baudrate = 38400 // Default for Daisy
	}

	// Check if device exists
	if _, err := os.Stat(device); err != nil {
		if globalSettings.DEBUG {
			log.Printf("Daisy AIS: device %s not found\n", device)
		}
		return false
	}

	// Open serial port
	daisySerialConfig = &serial.Config{
		Name:        device,
		Baud:        baudrate,
		ReadTimeout: 5 * time.Second,
	}

	p, err := serial.OpenPort(daisySerialConfig)
	if err != nil {
		log.Printf("Daisy AIS: error opening serial port %s: %s\n", device, err.Error())
		return false
	}

	daisySerialPort = p
	return true
}

// closeDaisySerial closes the serial port
func closeDaisySerial() {
	if daisySerialPort != nil {
		daisySerialPort.Close()
		daisySerialPort = nil
	}
	globalStatus.DaisyAIS_connected = false
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

// initDaisyAIS initializes the Daisy AIS subsystem
func initDaisyAIS() {
	go daisyAISListen()
}
