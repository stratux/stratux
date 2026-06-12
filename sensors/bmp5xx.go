package sensors

import (
	"sync"
	"time"

	"github.com/kidoman/embd"
	"github.com/stratux/stratux/sensors/bmp5xx"
)

type BMP5XX struct {
	sensor      *bmp5xx.BMP5XX
	temperature float64
	pressure    float64
	running     bool
	mu          sync.RWMutex
}

func NewBMP5XX(i2cbus *embd.I2CBus) (*BMP5XX, error) {
	var bmp *bmp5xx.BMP5XX

	for _, address := range []byte{bmp5xx.Address1, bmp5xx.Address2} {
		candidate := &bmp5xx.BMP5XX{Address: address, Bus: i2cbus}
		if candidate.Connected() {
			bmp = candidate
			break
		}
	}

	if bmp == nil {
		return nil, bmp5xx.ErrNotConnected
	}
	if err := bmp.Configure(); err != nil {
		return nil, err
	}

	newBmp := &BMP5XX{sensor: bmp, running: true}
	go newBmp.run()
	return newBmp, nil
}

func (bmp *BMP5XX) run() {
	clock := time.NewTicker(100 * time.Millisecond)
	defer clock.Stop()

	for range clock.C {
		bmp.mu.RLock()
		running := bmp.running
		bmp.mu.RUnlock()
		if !running {
			return
		}

		press, pressErr := bmp.sensor.ReadPressure()
		temp, tempErr := bmp.sensor.ReadTemperature()
		if pressErr != nil || tempErr != nil {
			bmp.mu.Lock()
			bmp.pressure = 0
			bmp.temperature = 0
			bmp.mu.Unlock()
			continue
		}

		bmp.mu.Lock()
		bmp.pressure = press
		bmp.temperature = temp
		bmp.mu.Unlock()
	}
}

func (bmp *BMP5XX) Close() {
	bmp.mu.Lock()
	bmp.running = false
	bmp.mu.Unlock()
	bmp.sensor.Close()
}

func (bmp *BMP5XX) Temperature() (float64, error) {
	bmp.mu.RLock()
	defer bmp.mu.RUnlock()
	if !bmp.running {
		return 0, bmp5xx.ErrNotConnected
	}

	return bmp.temperature, nil
}

func (bmp *BMP5XX) Pressure() (float64, error) {
	bmp.mu.RLock()
	defer bmp.mu.RUnlock()
	if !bmp.running {
		return 0, bmp5xx.ErrNotConnected
	}

	return bmp.pressure, nil
}
