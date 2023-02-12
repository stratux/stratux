/*
	Copyright (c) 2015-2016 Christopher Young,
	Copyright (c) 2022 Refactored R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	gps.go: GPS functions, GPS init, AHRS status messages, other external sensor monitoring.
*/

package gps

import (
	"bufio"
	"errors"
	"fmt"
	"log"
	"os"
	"strings"
	"time"

	"github.com/b3nn0/stratux/v2/common"
	"github.com/tarm/serial"
	"go.uber.org/ratelimit"
)

type SerialDiscoveryConfig struct {
	serialPort 		string
	name	   		string
	baudRate   		[]int
	timeOffsetPPS	time.Duration
	deviceType		uint
	reOpenPort		bool
	afterConnectFunc func(p *serial.Port)
}

type SerialGPSDevice struct {
	DEBUG                bool
	rxMessageCh  		 chan<- RXMessage
	eh                   *common.ExitHelper
}

func NewSerialGPSDevice(rxMessageCh chan<- RXMessage, debug bool) SerialGPSDevice {
	m := SerialGPSDevice{
		DEBUG:                debug,
		rxMessageCh:  		  rxMessageCh,
		eh:                   common.NewExitHelper(),
	}
	return m
}

func deviceDiscoveryConfig() []SerialDiscoveryConfig {
	all := make([]SerialDiscoveryConfig, 0)
	all = append(all, SerialDiscoveryConfig{
		serialPort: "/dev/ublox9", 
		name: "ublox9", 
		baudRate: []int{115200, 9600}, 
		timeOffsetPPS: 100 * time.Millisecond, 
		deviceType: GPS_TYPE_UBX9, 
		reOpenPort: true,
		afterConnectFunc:writeUblox9ConfigCommands})
	all = append(all, SerialDiscoveryConfig{
		serialPort: "/dev/ublox8", 
		name: "ublox8", 
		baudRate: []int{115200, 9600}, 
		timeOffsetPPS: 43 * time.Millisecond,  // RVT Calibrated
		deviceType: GPS_TYPE_UBX8,
		reOpenPort: true,
		afterConnectFunc: writeUblox8ConfigCommands,})
	all = append(all, SerialDiscoveryConfig{
		serialPort: "/dev/ublox7", 
		name: "ublox7", 
		baudRate: []int{115200, 9600},
		timeOffsetPPS: 100 * time.Millisecond, 
		deviceType: GPS_TYPE_UBX7,
		reOpenPort: true,
		afterConnectFunc: writeUblox6_7ConfigCommands})
	all = append(all, SerialDiscoveryConfig{
		serialPort: "/dev/ublox6", 
		name: "ublox6", 
		baudRate: []int{115200, 9600},
		timeOffsetPPS: 100 * time.Millisecond, 
		deviceType: GPS_TYPE_UBX6,
		reOpenPort: true,
		afterConnectFunc: writeUblox6_7ConfigCommands})
	// all = append(all, SerialDiscoveryConfig{
	// 	serialPort: "/dev/serialin", 
	// 	name: "serialIn", 
	// 	baudRate: []int{115200, 9600, 38400}, 
	// 	timeOffsetPPS: 100 * time.Millisecond,
	// 	deviceType: GPS_TYPE_SERIAL,
	// 	afterConnectFunc: writeSerialNOPCommands})
	all = append(all, SerialDiscoveryConfig{
		serialPort: "/dev/softrf_dongle", 
		name: "softrf_dongle", 
		baudRate: []int{115200}, 
		timeOffsetPPS: 100 * time.Millisecond,
		deviceType: GPS_TYPE_SOFTRF_DONGLE,
		afterConnectFunc: writeSoftRFDongleConfigCommands})
	for i := 0; i < 2; i++ {
		port := fmt.Sprintf("prolific%d", i)
		all = append(all, SerialDiscoveryConfig{
			serialPort: "/dev/"+port, 
			name: port, 
			baudRate: []int{4800, 38400, 9600}, 
			timeOffsetPPS: 100 * time.Millisecond,
			deviceType: GPS_TYPE_PROLIFIC,
			afterConnectFunc: writeProlificConfigCommands})
	}
	for i := 0; i < 10; i++ {
		port := fmt.Sprintf("ttyUSB%d", i)
		all = append(all, SerialDiscoveryConfig{
			serialPort: "/dev/"+port, 
			name: port, 
			baudRate: []int{115200, 9600, 38400}, 
			timeOffsetPPS: 100 * time.Millisecond,
			deviceType: GPS_TYPE_SERIAL,
			afterConnectFunc: writeSerialNOPCommands})
	}
	for i := 0; i < 10; i++ {
		port := fmt.Sprintf("ttyAMA%d", i)
		all = append(all, SerialDiscoveryConfig{
			serialPort: "/dev/"+port,  // Arduino and alike appears here
			name: port, 
			baudRate: []int{115200, 9600, 38400}, 
			timeOffsetPPS: 100 * time.Millisecond, 
			deviceType: GPS_TYPE_UART,
			afterConnectFunc: writeUARTConfigCommands})
	}		
	for i := 0; i < 10; i++ {
		port := fmt.Sprintf("ttyACM%d", i)
		all = append(all, SerialDiscoveryConfig{
			serialPort: "/dev/"+port,  // Arduino and alike appears here
			name: port, 
			baudRate: []int{115200, 9600, 38400}, 
			timeOffsetPPS: 100 * time.Millisecond, 
			deviceType: GPS_TYPE_SERIAL,
			afterConnectFunc: writeUARTConfigCommands})
	}
	return all
}

/*
u-blox5_Referenzmanual.pdf
Platform settings
Airborne <2g Recommended for typical airborne environment. No 2D position fixes supported.
p.91 - CFG-MSG
Navigation/Measurement Rate Settings
Header 0xB5 0x62
ID 0x06 0x08
0x0064 (100 ms)
0x0001
0x0001 (GPS time)
{0xB5, 0x62, 0x06, 0x08, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01}
p.109 CFG-NAV5 (0x06 0x24)
Poll Navigation Engine Settings
*/

/*
	chksumUBX()
		returns the two-byte Fletcher algorithm checksum of byte array msg.
		This is used in configuration messages for the u-blox  See p. 97 of the
		u-blox M8 Receiver Description.
*/

func chksumUBX(msg []byte) []byte {
	ret := make([]byte, 2)
	for i := 0; i < len(msg); i++ {
		ret[0] = ret[0] + msg[i]
		ret[1] = ret[1] + ret[0]
	}
	return ret
}

/*
	makeUBXCFG()
		creates a UBX-formatted package consisting of two sync characters,
		class, ID, payload length in bytes (2-byte little endian), payload, and checksum.
		See p. 95 of the u-blox M8 Receiver Description.
*/
func makeUBXCFG(class, id byte, msglen uint16, msg []byte) []byte {
	ret := make([]byte, 6)
	ret[0] = 0xB5
	ret[1] = 0x62
	ret[2] = class
	ret[3] = id
	ret[4] = byte(msglen & 0xFF)
	ret[5] = byte((msglen >> 8) & 0xFF)
	ret = append(ret, msg...)
	chk := chksumUBX(ret[2:])
	ret = append(ret, chk[0])
	ret = append(ret, chk[1])
	return ret
}

func (s *SerialGPSDevice) detectAndOpenSerialPort(device SerialDiscoveryConfig) (*(serial.Port)) {
	rl := ratelimit.New(1, ratelimit.Per(2*time.Second))
	for _, baud := range device.baudRate {
		// test if serial port exists on OS level
		if _, err := os.Stat(device.serialPort); err != nil { 
			continue
		}

		rl.Take()

		// If it exists, try to use it
		serialConfig := serial.Config{Name: device.serialPort, Baud: baud, ReadTimeout: time.Millisecond * 2500}
		p, err := serial.OpenPort(&serialConfig)
		if err != nil {
			continue
		}
		// If it works, try to read NMEA data from it if we find NEMA we are connected
		buffer := make([]byte, 10000)
		n, err := p.Read(buffer)
		if (n!=0 && err==nil) {
			splitted := strings.Split(string(buffer), "\n")			
			for _, line := range splitted {
				_, validNMEAcs := common.ValidateNMEAChecksum(line)
				if validNMEAcs {
					 log.Printf("Detected serial port %s with baud %d", device.serialPort, baud)
					return p
				}
			}
		}
		p.Close()
	}
	return nil
}

func writeSerialNOPCommands(p * serial.Port) {
	// NOP
}

func writeSoftRFDongleConfigCommands(p *serial.Port) {
	p.Write([]byte("@GNS 0x7\r\n"))    // enable SBAS
	p.Flush()
	time.Sleep(250 * time.Millisecond) // Otherwise second command doesn't seem to work?
	p.Write([]byte("@BSSL 0x2D\r\n"))  // enable GNGSV
	p.Flush()
}

func writeUARTConfigCommands(p *serial.Port) {
	writeUblox8ConfigCommands(p)
}

func writeProlificConfigCommands(p *serial.Port) {
	// Enable 5Hz. (To switch back to 1Hz: $PSRF103,00,7,00,0*22)
	p.Write(common.MakeNMEACmd("PSRF103,00,6,00,0"))
	// Enable GGA.
	p.Write(common.MakeNMEACmd("PSRF103,00,00,01,01"))
	// Enable GSA.
	p.Write(common.MakeNMEACmd("PSRF103,02,00,01,01"))
	// Enable RMC.
	p.Write(common.MakeNMEACmd("PSRF103,04,00,01,01"))
	// Enable VTG.
	p.Write(common.MakeNMEACmd("PSRF103,05,00,01,01"))
	// Enable GSV (once every 5 position updates)
	p.Write(common.MakeNMEACmd("PSRF103,03,00,05,01"))
	// Enable 38400 baud.
	p.Write(common.MakeNMEACmd("PSRF100,1,38400,8,1,0"))
}

func writeUblox6_7_8_9ConfigCommands(p *serial.Port) {
	// Reconfigure serial port.
	cfg := make([]byte, 20)
	cfg[0] = 0x01 // portID.
	cfg[1] = 0x00 // res0.
	cfg[2] = 0x00 // res1.
	cfg[3] = 0x00 // res1.

		
	//      [   7   ] [   6   ] [   5   ] [   4   ]
	//	0000 0000 0000 0000 0000 10x0 1100 0000
	// UART mode. 0 stop bits, no parity, 8 data bits. Little endian order.
	cfg[4] = 0xC0
	cfg[5] = 0x08
	cfg[6] = 0x00
	cfg[7] = 0x00

	// Baud rate. Little endian order.
	bdrt := uint32(115200)
	cfg[11] = byte((bdrt >> 24) & 0xFF)
	cfg[10] = byte((bdrt >> 16) & 0xFF)
	cfg[9] = byte((bdrt >> 8) & 0xFF)
	cfg[8] = byte(bdrt & 0xFF)

	// inProtoMask. NMEA and UBX. Little endian.
	cfg[12] = 0x03
	cfg[13] = 0x00

	// outProtoMask. NMEA. Little endian.
	cfg[14] = 0x02
	cfg[15] = 0x00

	cfg[16] = 0x00 // flags.
	cfg[17] = 0x00 // flags.

	cfg[18] = 0x00 //pad.
	cfg[19] = 0x00 //pad.

	// UBX-CFG-PRT (Port Configuration for UART)
	p.Write(makeUBXCFG(0x06, 0x00, 20, cfg))
}

func writeUblox6_7ConfigCommands(p *serial.Port) {
	cfgGnss := []byte{0x00, 0x00, 0xFF, 0x04} // numTrkChUse=0xFF: number of tracking channels to use will be set to number of tracking channels available in hardware
	gps     := []byte{0x00, 0x04, 0xFF, 0x00, 0x01, 0x00, 0x01, 0x01} // enable GPS with 4-255 channels (ublox default)
	sbas    := []byte{0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01} // enable SBAS with 1-3 channels (ublox default)
	qzss    := []byte{0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01} // enable QZSS with 0-3 channel (ublox default)
	glonass := []byte{0x06, 0x08, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x01} // disable GLONASS (ublox default)
	cfgGnss = append(cfgGnss, gps...)
	cfgGnss = append(cfgGnss, sbas...)
	cfgGnss = append(cfgGnss, qzss...)
	cfgGnss = append(cfgGnss, glonass...)
	p.Write(makeUBXCFG(0x06, 0x3E, uint16(len(cfgGnss)), cfgGnss))
	// Further generic commands
	writeUbloxGenericCommands(10, p)
	writeUblox6_7_8_9ConfigCommands(p)
}

func writeUblox8ConfigCommands(p *serial.Port) {
	cfgGnss := []byte{0x00, 0x00, 0xFF, 0x05}                         // numTrkChUse=0xFF: number of tracking channels to use will be set to number of tracking channels available in hardware
	gps := []byte{0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01}     // enable GPS with 8-16 channels (ublox default)
	sbas := []byte{0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01}    // enable SBAS with 1-3 channels (ublox default)
	galileo := []byte{0x02, 0x08, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01} // enable Galileo with 8-8 channels (ublox default: disabled and 4-8 channels)
	beidou := []byte{0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01}  // disable BEIDOU
	qzss := []byte{0x05, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01}    // enable QZSS 1-3 channels, L1C/A (ublox default: 0-3 channels)
	glonass := []byte{0x06, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01} // enable GLONASS with 8-16 channels (ublox default: 8-14 channels)

	cfgGnss = append(cfgGnss, gps...)
	cfgGnss = append(cfgGnss, sbas...)
	cfgGnss = append(cfgGnss, beidou...)
	cfgGnss = append(cfgGnss, qzss...)
	cfgGnss = append(cfgGnss, glonass...)
	p.Write(makeUBXCFG(0x06, 0x3E, uint16(len(cfgGnss)), cfgGnss)) // Succeeds on all chips supporting GPS+GLO

	cfgGnss[3] = 0x06
	cfgGnss = append(cfgGnss, galileo...)
	p.Write(makeUBXCFG(0x06, 0x3E, uint16(len(cfgGnss)), cfgGnss)) // Succeeds only on chips that support GPS+GLO+GAL
	// Further generic commands
	writeUbloxGenericCommands(10, p)
	writeUblox6_7_8_9ConfigCommands(p)
}

func writeUblox9ConfigCommands(p *serial.Port) {
	cfgGnss := []byte{0x00, 0x00, 0xFF, 0x06}                         // numTrkChUse=0xFF: number of tracking channels to use will be set to number of tracking channels available in hardware
	gps := []byte{0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01}     // enable GPS with 8-16 channels (ublox default)
	sbas := []byte{0x01, 0x03, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01}    // enable SBAS with 3-3 channels (ublox default)
	galileo := []byte{0x02, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01} // enable Galileo with 8-16 channels (ublox default: 8-12 channels)
	beidou := []byte{0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01}  // enable BEIDOU with 8-16 channels (ublox default: 2-5 channels)
	qzss := []byte{0x05, 0x03, 0x04, 0x00, 0x01, 0x00, 0x05, 0x01}    // enable QZSS 3-4 channels, L1C/A & L1S (ublox default)
	glonass := []byte{0x06, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01} // enable GLONASS with 8-16 tracking channels (ublox default: 8-12 channels)

	cfgGnss = append(cfgGnss, gps...)
	cfgGnss = append(cfgGnss, sbas...)
	cfgGnss = append(cfgGnss, beidou...)
	cfgGnss = append(cfgGnss, qzss...)
	cfgGnss = append(cfgGnss, glonass...)
	cfgGnss = append(cfgGnss, galileo...)
	p.Write(makeUBXCFG(0x06, 0x3E, uint16(len(cfgGnss)), cfgGnss))
	writeUbloxGenericCommands(10, p)
	writeUblox6_7_8_9ConfigCommands(p)
}

func writeUbloxGenericCommands(navrate uint16, p *serial.Port) {

	// UBX-CFG-NMEA (change NMEA protocol version to 4.0 extended)
	p.Write(makeUBXCFG(0x06, 0x17, 20, []byte{0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}))

	// UBX-CFG-PMS
	p.Write(makeUBXCFG(0x06, 0x86, 8, []byte{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // Full Power Mode
	// p.Write(makeUBXCFG(0x06, 0x86, 8, []byte{0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // Balanced Power Mode

	// UBX-CFG-NAV5                           |mask1...|  dyn
	p.Write(makeUBXCFG(0x06, 0x24, 36, []byte{0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // Dynamic platform model: airborne with <2g acceleration

	// UBX-CFG-SBAS (disable integrity, enable auto-scan)
	p.Write(makeUBXCFG(0x06, 0x16, 8, []byte{0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}))

	// UBX-CFG-MSG (NMEA Standard Messages)  msg   msg   Ports 1-6 (every 10th message over UART1, every message over USB)
	//                                       Class ID    I2C   UART1 UART2 USB   SPI   Res
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00})) // GGA - Global positioning system fix data
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // GLL - Latitude and longitude, with time of position fix and status
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x02, 0x00, 0x05, 0x00, 0x05, 0x00, 0x00})) // GSA - GNSS DOP and Active Satellites
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x03, 0x00, 0x05, 0x00, 0x05, 0x00, 0x00})) // GSV - GNSS Satellites in View
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00})) // RMC - Recommended Minimum data
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x05, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00})) // VGT - Course over ground and Ground speed
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // GRS - GNSS Range Residuals
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // GST - GNSS Pseudo Range Error Statistics
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // ZDA - Time and Date<
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // GBS - GNSS Satellite Fault Detection
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // DTM - Datum Reference
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // GNS - GNSS fix data
	// p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // ???
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // VLW - Dual ground/water distance

	// UBX-CFG-MSG (NMEA PUBX Messages)      msg   msg   Ports 1-6
	//                                       Class ID    I2C   UART1 UART2 USB   SPI   Res
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // Ublox - Lat/Long Position Data
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // Ublox - Satellite Status
	p.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})) // Ublox - Time of Day and Clock Information

	if navrate == 10 {
		p.Write(makeUBXCFG(0x06, 0x08, 6, []byte{0x64, 0x00, 0x01, 0x00, 0x01, 0x00})) // 100ms & 1 cycle -> 10Hz (UBX-CFG-RATE payload bytes: little endian!)
	} else if navrate == 5 {
		p.Write(makeUBXCFG(0x06, 0x08, 6, []byte{0xC8, 0x00, 0x01, 0x00, 0x01, 0x00})) // 200ms & 1 cycle -> 5Hz (UBX-CFG-RATE payload bytes: little endian!)
	} else if navrate == 2 {
		p.Write(makeUBXCFG(0x06, 0x08, 6, []byte{0xF4, 0x01, 0x01, 0x00, 0x01, 0x00})) // 500ms & 1 cycle -> 2Hz (UBX-CFG-RATE payload bytes: little endian!)
	} else if navrate == 1 {
		p.Write(makeUBXCFG(0x06, 0x08, 6, []byte{0xE8, 0x03, 0x01, 0x00, 0x01, 0x00})) // 1000ms & 1 cycle -> 1Hz (UBX-CFG-RATE payload bytes: little endian!)
	}
}

/**
run the main serial reader and writer on one single connected GPS
It will send RXMessages the rxMessageCh, it will send any gpsMessage on the txChannel back to the attached GPS
and will send out discovery messages
When a OGN tracker is detecdted it will configure as ublox8
*/
func (s *SerialGPSDevice) serialRXTX(device SerialDiscoveryConfig) error {

	serialPort := s.detectAndOpenSerialPort(device)
	if serialPort != nil {

		// Close and Re-Open if requested
		// The GPS might have been configured on a different baudrate after calling afterConnectFunc
		if device.afterConnectFunc!=nil && device.reOpenPort {
			device.afterConnectFunc(serialPort)
			serialPort.Close()
			serialPort = s.detectAndOpenSerialPort(device)
			if (serialPort == nil) {
				return errors.New("Failed to detect serial port after initialisation")
			}
		}
		s.eh.Add()
		defer s.eh.Done()

		go func() {
			<- s.eh.C
			serialPort.Close()
			GetServiceDiscovery().Connected(device.name, false)
		}()

		log.Printf("Found GPS device %s\n", device.name)

		ognTrackerConfigured := false

		TXChannel := make(chan []byte, 10) // Create a unblocking channel to receive TX messages on to send to GPS
		GetServiceDiscovery().Send(DiscoveredDevice{
			Name:               device.name,
			Content:			CONTENT_TX_CHANNEL | CONTENT_TYPE | CONTENT_SOURCE | CONTENT_OFFSET_PPS | CONTENT_CONNECTED,
			Connected: 			true,
			GPSDetectedType:    device.deviceType,
			GPSSource:          GPS_SOURCE_SERIAL,
			TXChannel:          TXChannel,
			GPSTimeOffsetPPS:   device.timeOffsetPPS,
		})

		// Blocking function that reads serial data
		serialReader := func() {
			i := 0 //debug monitor
			scanner := bufio.NewScanner(serialPort)
			for scanner.Scan() {
				
				if scanner.Err() != nil {
					return
				}
				
				i++
				if s.DEBUG && i%100 == 0 {
					log.Printf("scanner loop iteration i=%d\n", i) // debug monitor
				}

				nmeaLine := scanner.Text()
				startIdx := strings.Index(nmeaLine, "$")
				if startIdx < 0 {
					continue
				}

				nmeaLine = nmeaLine[startIdx:]

				// We peek into the NMEA string, if we detect OGN for the first time we configure it as a OGN device
				if !ognTrackerConfigured && strings.HasPrefix(nmeaLine, "$POGNR") {
					ognTrackerConfigured = true
					go func() {
						log.Printf("OGN detected, configuring with Ublox8 config\n")
						writeUblox8ConfigCommands(serialPort)
						serialPort.Flush()
						//time.Sleep(time.Second * 5)
						// Generic commands always seems to return in a invalid NMEA string, hope that is fine?
						// writeUbloxGenericCommands(5, serialPort)
						//serialPort.Flush()
						// Notify of this device type
						GetServiceDiscovery().Send(DiscoveredDevice{
							Name:               device.name,
							Content:			CONTENT_TYPE | CONTENT_OFFSET_PPS,
							GPSDetectedType:    GPS_TYPE_OGNTRACKER,
							GPSTimeOffsetPPS:   150 * time.Millisecond,
						})
					}()
				}
				
				select {
				case s.rxMessageCh <- RXMessage {
					Name:     device.name,
					NmeaLine: nmeaLine,
				}:
				default:
					log.Printf("Serial rxMessageCh Full")
				}
			}

			if s.DEBUG {
				log.Printf("Exiting serialGPSReader() after i=%d loops\n", i) // debug monitor
			}
		}

		// We use a private ExitHelper for the local Writer because when serialReader stops we also want the serialWriter to quit
		localQh := common.NewExitHelper()
		defer localQh.Exit()

		serialWriter := func() {
			localQh.Add()
			defer localQh.Done()
			// Rate limited to ensure we only do 2 messages per second over serial port
			// We currently assume we will never send a lot of commands to any serial device
			rl := ratelimit.New(1, ratelimit.Per(4*time.Second))
			for {
				select {
				case <-localQh.C:
					return
				case txMessage := <-TXChannel:
					rl.Take()
					serialPort.Write(txMessage)
					serialPort.Flush()
				}
			}
		}

		go serialWriter()
		serialReader()
		log.Printf("serialGPSDevice: Stopping %s", device.name)
	}

	return nil;
}

/**
Discover serial devices
*/
func (s *SerialGPSDevice) deviceDiscovery() {
	s.eh.Add()
	defer s.eh.Done()

	devices := deviceDiscoveryConfig()
	currentDevice := 0
	connectedDevices := make(map[string]bool)

	scanNextDevice := func () {
		// Take device and test if this device is already connected, if not then try to connect
		device := devices[currentDevice]
		if s.DEBUG {
			log.Printf("Trying serial GPS device %s\n", device.name)
		}
		if connectedDevices[device.name] == false {
			connectedDevices[device.name] = true
			go func() {
				if error := s.serialRXTX(device); error!=nil {
					log.Printf("Error connecting to %s: %s\n", device.name, error.Error())
				}
				connectedDevices[device.name] = false
			}()
		}
		currentDevice++
		if (currentDevice >= len(devices)) {
			currentDevice = 0
		}
	}

	timer := time.NewTicker(1 * time.Second)
	for {
		select {
		case <- s.eh.C:
			return
		case <- timer.C:
			scanNextDevice()
		}
	}
}

/**
Request to stop all goroutines and stop serial/GPS
*/
func (s *SerialGPSDevice) Stop() {
	log.Printf("Stopping Serial service")
	s.eh.Exit()
	log.Printf("... Serial service stopped")
}

func (n *SerialGPSDevice) Scan(leh *common.ExitHelper) {
	// Not implemented
}

/**
Start the serial GPS system and start logging for devices. When found device discovery will send out message and NMEA lines will poor in
Main tasks is configure the GPS/Location services and maintain a stable serial/USB connection
*/
func (s *SerialGPSDevice) Run() {
	go s.deviceDiscovery()
	log.Printf("serialGPSDevice: Run: Enabled USB/Serial devices")
}
