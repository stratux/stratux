// Package bmp5xx provides a small I2C driver for Bosch BMP58x pressure sensors.
//
// Register definitions and data conversion are based on Bosch Sensortec's
// BMP5 Sensor API:
// https://github.com/boschsensortec/BMP5_SensorAPI
package bmp5xx

import "errors"

const (
	Address1 byte = 0x46
	Address2 byte = 0x47
)

const (
	RegChipId    byte = 0x01
	RegStatus    byte = 0x28
	RegOsrConfig byte = 0x36
	RegOdrConfig byte = 0x37
	RegCmd       byte = 0x7E

	RegTemperatureData byte = 0x1D
	RegPressureData    byte = 0x20
)

const (
	ChipIdBMP580 byte = 0x50
	ChipIdBMP581 byte = 0x50
	ChipIdBMP585 byte = 0x51
	ChipIdBMP587 byte = 0x51

	SoftReset byte = 0xB6
)

const (
	osrTemperature8X byte = 0x03
	osrPressure2X    byte = 0x01 << 3
	osrPressureEn    byte = 0x01 << 6

	odr10Hz             byte = 0x17 << 2
	powerModeNormal     byte = 0x01
	deepStandbyDisabled byte = 0x01 << 7
)

var (
	ErrNotConnected = errors.New("bmp5xx: not connected")
	errConfigWrite  = errors.New("bmp5xx: failed to configure sensor, check connection")
	errDataRead     = errors.New("bmp5xx: failed to read sensor data")
)

