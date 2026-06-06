package bmp5xx

import (
	"time"

	"github.com/kidoman/embd"
)

type BMP5XX struct {
	Bus     *embd.I2CBus
	Address byte
}

func (d *BMP5XX) Connected() bool {
	chipID, err := d.readByte(RegChipId)
	return err == nil && isSupportedChipID(chipID)
}

func (d *BMP5XX) Configure() error {
	if !d.Connected() {
		return ErrNotConnected
	}

	if err := d.writeRegister(RegOsrConfig, osrTemperature8X|osrPressure2X|osrPressureEn); err != nil {
		return errConfigWrite
	}
	if err := d.writeRegister(RegOdrConfig, odr10Hz|powerModeNormal|deepStandbyDisabled); err != nil {
		return errConfigWrite
	}

	time.Sleep(5 * time.Millisecond)
	return nil
}

func (d *BMP5XX) ReadTemperature() (float64, error) {
	data, err := d.readRegister(RegTemperatureData, 3)
	if err != nil {
		return 0, errDataRead
	}

	raw := signExtend24(data)
	return float64(raw) / 65536.0, nil
}

func (d *BMP5XX) ReadPressure() (float64, error) {
	data, err := d.readRegister(RegPressureData, 3)
	if err != nil {
		return 0, errDataRead
	}

	raw := uint32(data[2])<<16 | uint32(data[1])<<8 | uint32(data[0])
	return float64(raw) / 6400.0, nil
}

func (d *BMP5XX) Close() {
	_ = d.writeRegister(RegOdrConfig, 0)
}

func isSupportedChipID(chipID byte) bool {
	return chipID == ChipIdBMP580 || chipID == ChipIdBMP585
}

func signExtend24(data []byte) int32 {
	raw := int32(uint32(data[2])<<16 | uint32(data[1])<<8 | uint32(data[0]))
	if raw&0x800000 != 0 {
		raw |= ^0xFFFFFF
	}
	return raw
}

func (d *BMP5XX) readRegister(register byte, count int) ([]byte, error) {
	data := make([]byte, count)
	err := (*d.Bus).ReadFromReg(d.Address, register, data)
	return data, err
}

func (d *BMP5XX) readByte(register byte) (byte, error) {
	data, err := d.readRegister(register, 1)
	if err != nil {
		return 0, err
	}
	return data[0], nil
}

func (d *BMP5XX) writeRegister(register byte, data byte) error {
	return (*d.Bus).WriteToReg(d.Address, register, []byte{data})
}
