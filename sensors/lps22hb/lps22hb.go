package lps22hb

import (
	"errors"

	"github.com/kidoman/embd"
)

var (
	ErrNotConnected = errors.New("lps22hb: sensor not connected")
	ErrConfigWrite  = errors.New("lps22hb: failed to configure sensor")
	ErrReadFailed   = errors.New("lps22hb: failed to read data")
)

// LPS22HB represents the pressure sensor
type LPS22HB struct {
	Bus     *embd.I2CBus
	Address byte
}

// NewLPS22HB creates a new LPS22HB sensor instance
func NewLPS22HB(bus *embd.I2CBus, address byte) *LPS22HB {
	return &LPS22HB{
		Bus:     bus,
		Address: address,
	}
}

// Connected checks if the sensor is responding with correct chip ID
func (d *LPS22HB) Connected() bool {
	data, err := d.readRegister(RegWhoAmI, 1)
	return err == nil && data[0] == ChipId
}

// Configure initializes the sensor with default settings
// Sets 25Hz ODR with block data update enabled
func (d *LPS22HB) Configure() error {
	if !d.Connected() {
		return ErrNotConnected
	}

	// Software reset first
	err := d.writeRegister(RegCtrlReg2, SwReset)
	if err != nil {
		return ErrConfigWrite
	}

	// Wait for reset to complete (check if SwReset bit is cleared)
	for i := 0; i < 10; i++ {
		data, err := d.readRegister(RegCtrlReg2, 1)
		if err == nil && (data[0]&SwReset) == 0 {
			break
		}
	}

	// Enable auto-increment for multi-byte reads
	err = d.writeRegister(RegCtrlReg2, AutoIncr)
	if err != nil {
		return ErrConfigWrite
	}

	// Set ODR to 25Hz with BDU enabled
	err = d.writeRegister(RegCtrlReg1, Odr25Hz|BduEnable)
	if err != nil {
		return ErrConfigWrite
	}

	return nil
}

// ReadPressure returns the pressure in mbar (hPa)
func (d *LPS22HB) ReadPressure() (float64, error) {
	// Read 3 bytes starting from PRESS_OUT_XL (auto-increment enabled)
	data, err := d.readRegister(RegPressOutXL, 3)
	if err != nil {
		return 0, ErrReadFailed
	}

	// Combine bytes into 24-bit signed value
	raw := int32(data[0]) | int32(data[1])<<8 | int32(data[2])<<16

	// Sign extension for negative values
	if raw >= 0x800000 {
		raw -= 0x1000000
	}

	// Convert to mbar: divide by 4096 (as per datasheet)
	pressure := float64(raw) / 4096.0

	return pressure, nil
}

// ReadTemperature returns the temperature in degrees Celsius
func (d *LPS22HB) ReadTemperature() (float64, error) {
	// Read 2 bytes starting from TEMP_OUT_L (auto-increment enabled)
	data, err := d.readRegister(RegTempOutL, 2)
	if err != nil {
		return 0, ErrReadFailed
	}

	// Combine bytes into 16-bit signed value
	raw := int16(data[0]) | int16(data[1])<<8

	// Convert to Celsius: divide by 100 (as per datasheet)
	temperature := float64(raw) / 100.0

	return temperature, nil
}

// PowerDown puts the sensor into power-down mode
func (d *LPS22HB) PowerDown() error {
	return d.writeRegister(RegCtrlReg1, OdrPowerDown)
}

func (d *LPS22HB) readRegister(register byte, length int) ([]byte, error) {
	data := make([]byte, length)
	err := (*d.Bus).ReadFromReg(d.Address, register, data)
	return data, err
}

func (d *LPS22HB) writeRegister(register byte, data byte) error {
	return (*d.Bus).WriteToReg(d.Address, register, []byte{data})
}
