package sensors

import (
	"time"

	"github.com/kidoman/embd"
	"github.com/stratux/stratux/sensors/lps22hb"
)

// LPS22HB wraps the LPS22HB sensor and implements PressureReader interface
type LPS22HB struct {
	sensor      *lps22hb.LPS22HB
	temperature float64
	pressure    float64
	running     bool
}

// NewLPS22HB creates a new LPS22HB pressure sensor reader
// Tries both possible I2C addresses (0x5C and 0x5D)
func NewLPS22HB(i2cbus *embd.I2CBus) (*LPS22HB, error) {
	// Try address 0x5C first (default for Sense HAT B)
	sensor := lps22hb.NewLPS22HB(i2cbus, lps22hb.Address1)

	var connected bool
	for n := 0; n < 5; n++ {
		if sensor.Connected() {
			connected = true
			break
		}
		time.Sleep(time.Millisecond * 10)
	}

	// Try alternate address if first didn't work
	if !connected {
		sensor = lps22hb.NewLPS22HB(i2cbus, lps22hb.Address2)
		for n := 0; n < 5; n++ {
			if sensor.Connected() {
				connected = true
				break
			}
			time.Sleep(time.Millisecond * 10)
		}
	}

	if !connected {
		return nil, lps22hb.ErrNotConnected
	}

	err := sensor.Configure()
	if err != nil {
		return nil, err
	}

	newSensor := &LPS22HB{sensor: sensor}
	go newSensor.run()

	return newSensor, nil
}

func (s *LPS22HB) run() {
	s.running = true
	clock := time.NewTicker(100 * time.Millisecond)
	for s.running {
		<-clock.C
		if p, err := s.sensor.ReadPressure(); err == nil {
			s.pressure = p
		}
		if t, err := s.sensor.ReadTemperature(); err == nil {
			s.temperature = t
		}
	}
}

// Temperature returns the current temperature in degrees Celsius
func (s *LPS22HB) Temperature() (float64, error) {
	if !s.running {
		return 0, lps22hb.ErrNotConnected
	}
	return s.temperature, nil
}

// Pressure returns the current pressure in mbar (hPa)
func (s *LPS22HB) Pressure() (float64, error) {
	if !s.running {
		return 0, lps22hb.ErrNotConnected
	}
	return s.pressure, nil
}

// Close stops the sensor readings and puts the sensor in power-down mode
func (s *LPS22HB) Close() {
	s.running = false
	s.sensor.PowerDown()
}
