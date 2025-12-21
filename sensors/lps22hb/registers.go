// Package lps22hb provides a driver for ST's LPS22HB digital pressure sensor.
// Datasheet: https://www.st.com/resource/en/datasheet/lps22hb.pdf
package lps22hb

// I2C addresses (depending on SA0 pin state)
const (
	Address1 byte = 0x5C // SA0 = 0 (default on Sense HAT B)
	Address2 byte = 0x5D // SA0 = 1
)

// Register addresses
const (
	RegWhoAmI    byte = 0x0F // Device identification register
	RegCtrlReg1  byte = 0x10 // Control register 1 (ODR, BDU, LPFP)
	RegCtrlReg2  byte = 0x11 // Control register 2 (boot, FIFO, one-shot, I2C disable)
	RegCtrlReg3  byte = 0x12 // Control register 3 (interrupt config)
	RegStatus    byte = 0x27 // Status register
	RegPressOutXL byte = 0x28 // Pressure output LSB
	RegPressOutL  byte = 0x29 // Pressure output middle byte
	RegPressOutH  byte = 0x2A // Pressure output MSB
	RegTempOutL   byte = 0x2B // Temperature output LSB
	RegTempOutH   byte = 0x2C // Temperature output MSB
)

// Device identification
const (
	ChipId byte = 0xB1 // Expected WHO_AM_I value for LPS22HB
)

// Control register 1 settings
const (
	// Output data rate (ODR) settings
	OdrPowerDown byte = 0x00 // Power-down mode
	Odr1Hz       byte = 0x10 // 1 Hz
	Odr10Hz      byte = 0x20 // 10 Hz
	Odr25Hz      byte = 0x30 // 25 Hz
	Odr50Hz      byte = 0x40 // 50 Hz
	Odr75Hz      byte = 0x50 // 75 Hz

	// Block data update (recommended to set)
	BduEnable byte = 0x02

	// Low-pass filter on pressure data
	LpfpDisable byte = 0x00
	LpfpEnable  byte = 0x08
)

// Control register 2 settings
const (
	OneShot   byte = 0x01 // Trigger single measurement
	SwReset   byte = 0x04 // Software reset
	AutoIncr  byte = 0x10 // Auto-increment register address
	Boot      byte = 0x80 // Reboot memory content
)

// Status register bits
const (
	StatusPressReady byte = 0x01 // New pressure data available
	StatusTempReady  byte = 0x02 // New temperature data available
)
