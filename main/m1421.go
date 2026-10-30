package main

import (
	"bufio"
	"encoding/hex"
	"fmt"
	"log"
	"os"
	"strings"
	"time"

	"github.com/stratux/serial"
)

var m1421SerialPort *serial.Port
var m1421SerialConfig *serial.Config
var m1421Device string // Will be set during auto-detection

const M1421_BAUD = 115200
const M1421_OUTPUT_FORMAT = "beast" // Beast binary format
const BEAST_ESC = 0x1A

// Common GPIO UART device paths to probe (in order of preference)
var m1421UARTPaths = []string{
	"/dev/ttyAMA0", // Primary UART on Pi 3/4/5
	"/dev/ttyS0",   // Alternative naming
	"/dev/serial0", // Symlink to primary UART
	"/dev/serial1", // Secondary UART (if available)
}

// Supported baud rates for auto-detection
var m1421BaudRates = []int{9600, 19200, 38400, 57600, 115200, 230400, 460800}

func initM1421Serial(isTraceReplayMode bool) error {
	if isTraceReplayMode {
		return nil
	}

	go func() {
		watchTicker := time.NewTicker(2 * time.Second)
		for {
			<-watchTicker.C

			// Watch for M1421 being enabled.
			if !globalSettings.M1421_Enabled || globalStatus.M1421_connected {
				// M1421 not enabled or already connected.
				continue
			}

			// Auto-detect M1421 on GPIO UART
			device, err := detectM1421Device()
			if err != nil {
				// Device not detected yet, continue trying
				continue
			}

			m1421Device = device
			log.Printf("===== M1421 Device found at %s =====\n", m1421Device)

			// Attempt to detect baud rate and configure M1421.
			if err := configureM1421(); err != nil {
				log.Printf("M1421 configuration failed: %s\n", err.Error())
				continue
			}

			log.Printf("M1421 init success.\n")
			globalStatus.M1421_connected = true

			// Start a goroutine to read Beast frames from the M1421.
			go m1421SerialPortReader(m1421SerialPort)
		}
	}()

	return nil
}

// detectM1421Device probes common GPIO UART paths to find the M1421.
// Returns the device path if found, error otherwise.
func detectM1421Device() (string, error) {
	for _, devicePath := range m1421UARTPaths {
		if _, err := os.Stat(devicePath); err != nil {
			// Device doesn't exist, try next
			continue
		}

		// Try to probe this UART at common baud rates
		for _, baud := range m1421BaudRates {
			config := &serial.Config{Name: devicePath, Baud: baud}
			port, err := serial.OpenPort(config)
			if err != nil {
				continue
			}

			// Send AT probe command
			if err := sendM1421Command(port, "AT"); err == nil {
				port.Close()
				log.Printf("M1421 detected at %s (%d baud)\n", devicePath, baud)
				return devicePath, nil
			}

			port.Close()
		}
	}

	return "", fmt.Errorf("M1421 not detected on any GPIO UART")
}

// configureM1421 attempts to auto-detect the M1421's current baud rate,
// then configures it to output Beast format at 115200 baud.
func configureM1421() error {
	var detectedBaud int
	var err error

	// Try to detect current baud rate
	log.Printf("Attempting to detect M1421 baud rate...\n")
	detectedBaud, err = detectM1421BaudRateOnDevice()
	if err != nil {
		log.Printf("Baud rate detection failed: %s, attempting default (%d)\n", err.Error(), M1421_BAUD)
		detectedBaud = M1421_BAUD
	} else {
		log.Printf("Detected M1421 baud rate: %d\n", detectedBaud)
	}

	// Open port at detected baud rate
	m1421SerialConfig = &serial.Config{Name: m1421Device, Baud: detectedBaud}
	port, err := serial.OpenPort(m1421SerialConfig)
	if err != nil {
		return fmt.Errorf("failed to open M1421 port at %s: %w", m1421Device, err)
	}
	defer port.Close()

	// Send AT commands to query/configure M1421
	log.Printf("Configuring M1421 (setting output format: %s, baud: %d)...\n", M1421_OUTPUT_FORMAT, M1421_BAUD)

	// Query device info
	if err := sendM1421Command(port, "AT+I?"); err != nil {
		log.Printf("Warning: Could not query device info: %s\n", err.Error())
	}

	// Configure output format to Beast
	if err := sendM1421Command(port, fmt.Sprintf("AT+OUTPUT=%s", M1421_OUTPUT_FORMAT)); err != nil {
		log.Printf("Warning: Could not set output format: %s\n", err.Error())
	}

	// Configure baud rate
	if detectedBaud != M1421_BAUD {
		if err := sendM1421Command(port, fmt.Sprintf("AT+BAUD=%d", M1421_BAUD)); err != nil {
			log.Printf("Warning: Could not set baud rate: %s\n", err.Error())
		}
		// Save configuration to flash
		if err := sendM1421Command(port, "AT&W"); err != nil {
			log.Printf("Warning: Could not save configuration: %s\n", err.Error())
		}
		// Give device time to reset
		time.Sleep(500 * time.Millisecond)
	}

	// Re-open at target baud rate
	m1421SerialConfig = &serial.Config{Name: m1421Device, Baud: M1421_BAUD}
	p, err := serial.OpenPort(m1421SerialConfig)
	if err != nil {
		return fmt.Errorf("failed to reopen M1421 at target baud rate: %w", err)
	}

	m1421SerialPort = p
	return nil
}

// detectM1421BaudRateOnDevice attempts to probe the M1421 at various baud rates
// by sending an AT command and looking for a valid response.
func detectM1421BaudRateOnDevice() (int, error) {
	for _, baud := range m1421BaudRates {
		config := &serial.Config{Name: m1421Device, Baud: baud}
		port, err := serial.OpenPort(config)
		if err != nil {
			continue
		}

		// Try sending AT command
		success := false
		if err := sendM1421Command(port, "AT"); err == nil {
			success = true
		}

		port.Close()

		if success {
			return baud, nil
		}
	}

	return 0, fmt.Errorf("could not detect M1421 baud rate")
}

// sendM1421Command sends an AT command to the M1421 and waits for a response.
// Returns nil if "OK" is received, error otherwise.
func sendM1421Command(port *serial.Port, cmd string) error {
	cmdWithNewline := cmd + "\r\n"
	if _, err := port.Write([]byte(cmdWithNewline)); err != nil {
		return fmt.Errorf("write failed: %w", err)
	}

	// Read response with timeout
	port.SetReadTimeout(500 * time.Millisecond)
	rdr := bufio.NewReader(port)

	for i := 0; i < 10; i++ { // Try reading up to 10 lines
		line, err := rdr.ReadString('\n')
		if err != nil {
			continue
		}

		line = strings.TrimSpace(line)
		if line == "OK" {
			return nil
		}
		if strings.Contains(line, "ERROR") {
			return fmt.Errorf("M1421 returned error: %s", line)
		}
	}

	return fmt.Errorf("no valid response from M1421")
}

// m1421SerialPortReader reads Beast-formatted frames from the M1421 serial port.
// Beast frame format:
//   - 0x1A (ESC) marks frame start
//   - 1 byte frame type/length indicator
//   - Payload (varies by type)
//   - 0x1A (ESC) marks frame end (or next frame start)
func m1421SerialPortReader(serialPort *serial.Port) {
	defer func() {
		globalStatus.M1421_connected = false
		serialPort.Close()
	}()

	tmpBuf := make([]byte, 4096)
	var buf []byte

	for {
		n, err := serialPort.Read(tmpBuf)
		if err != nil {
			log.Printf("M1421 serial port error, shutting down: %s\n", err.Error())
			return
		}

		buf = append(buf, tmpBuf[:n]...)

		// Process complete Beast frames in the buffer
		for {
			frame, remaining := extractBeastFrame(buf)
			if frame == nil {
				break // No complete frame yet
			}
			buf = remaining

			// Process the frame
			processM1421BeastFrame(frame)
		}
	}
}

// extractBeastFrame extracts one Beast frame from the buffer.
// Returns (frame, remaining buffer) or (nil, buf) if no complete frame.
func extractBeastFrame(buf []byte) ([]byte, []byte) {
	if len(buf) < 2 {
		return nil, buf
	}

	// Find first ESC byte
	startIdx := -1
	for i := 0; i < len(buf); i++ {
		if buf[i] == BEAST_ESC {
			startIdx = i
			break
		}
	}

	if startIdx == -1 {
		// No ESC found, discard buffer
		return nil, buf[len(buf):]
	}

	if startIdx > 0 {
		// Discard bytes before first ESC
		buf = buf[startIdx:]
	}

	if len(buf) < 2 {
		return nil, buf
	}

	// Frame type is the byte after ESC
	frameType := buf[1]

	// Determine expected payload length based on frame type
	var expectedLen int
	switch frameType {
	case 0x1A:
		// Escaped ESC byte (stuffing), not a frame start
		return nil, buf[1:]
	case 0x00: // Downlink frame (28 bytes) + ESC + frame type (already 2 bytes)
		expectedLen = 30
	case 0x01: // Downlink with DF11 (14 bytes)
		expectedLen = 16
	case 0x02: // Uplink frame (14 bytes)
		expectedLen = 16
	default:
		// Unknown frame type, skip this ESC
		return nil, buf[1:]
	}

	if len(buf) < expectedLen {
		// Not enough data yet
		return nil, buf
	}

	// Extract frame (including ESC markers)
	frame := buf[:expectedLen]
	remaining := buf[expectedLen:]

	return frame, remaining
}

// processM1421BeastFrame processes a Beast-formatted ADS-B frame from the M1421.
func processM1421BeastFrame(frame []byte) {
	if len(frame) < 2 {
		return
	}

	// Beast frame: ESC + type + payload + ESC (payload last byte is often the ESC)
	// Extract the message payload (skip ESC at start and type byte)
	frameType := frame[1]

	var msgLen int
	var msgStart int

	switch frameType {
	case 0x00: // DF0-4 or DF16-18 (long frame, 28 bytes + TS/RSSI)
		msgStart = 2
		msgLen = 14 // Standard ADS-B message is 14 bytes for long frame (84 bits / 8)
		// Note: Actual Beast payload is longer; extract just the message
		if len(frame) >= 30 {
			msgLen = 14
		}
	case 0x01: // DF11 or DF18 (short frame, 14 bytes)
		msgStart = 2
		msgLen = 7 // Short frame is 7 bytes
	case 0x02: // Uplink (14 bytes)
		msgStart = 2
		msgLen = 7
	default:
		log.Printf("Unknown Beast frame type: 0x%02x\n", frameType)
		return
	}

	if len(frame) < msgStart+msgLen {
		log.Printf("Beast frame too short for type 0x%02x\n", frameType)
		return
	}

	// Extract message and signal strength
	message := frame[msgStart : msgStart+msgLen]
	var rssi int8 = -50 // Default RSSI; may be encoded in frame

	// Extract RSSI if present (usually after message in Beast format)
	if len(frame) > msgStart+msgLen {
		// Try to extract RSSI from the message
		// In typical Beast mode: last 2 bytes may be TS/RSSI
		rssi = int8(frame[len(frame)-1])
	}

	// Format as ADS-B message for relay
	// Use "-" prefix for downlink (ADS-B), "+" for uplink
	msgPrefix := "-"
	if frameType == 0x02 {
		msgPrefix = "+"
	}

	msgHex := hex.EncodeToString(message)
	toRelay := fmt.Sprintf("%s%s;ss=%d;", msgPrefix, msgHex, rssi)

	// Parse and relay through Stratux pipeline
	o, msgtype := parseInput(toRelay)
	if o != nil && msgtype != 0 {
		relayMessage(msgtype, o)
	}
}
