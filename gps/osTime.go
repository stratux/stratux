package gps

import (
	"log"
	"math"
	"os/exec"
	"sync"
	"time"

	"github.com/b3nn0/stratux/v2/common"
)

// Small go package to set the system time
const SHOW_TIME_DIFFERENCE_ONLY = false                  // Use this to enable calbration mode if you want to add/test a new device pps
const AVERAGE_OVER = 10.0                                // Average over 10 seconds to to determine the average time difference
const ACCEPTABLE_TIME_OFFSET = 40 * time.Millisecond     // Number of ms we still accept as difference between GPS and OS time
const TOTAL_MEASURMENETS = 10                            // For calibration, the number of runs
const TIME_BETWEEN_MEASUREMENTS = 200 * time.Millisecond // For calibration the time between each run

type OSTimeSetter struct {
	movingTimeDifference  float64   // Keeps track on teh difference between GPS time and OS time
	timeToSetTime         float64   // Time it takes to set the system time from a given time, this is due to starting of a process in the OS
	lastMovingAverageTime time.Time // Last time when moving avergae time was set
	lastSetTime           time.Time // Last time when the time was set
	c                     chan setTime
	eh                   *common.ExitHelper
}

type setTime struct {
	t time.Time
	o time.Duration
}

var (
	onceOsTimeInstance sync.Once
	osTimeInstance     OSTimeSetter
)

func GetOSTimeSetter() *OSTimeSetter {

	onceOsTimeInstance.Do(func() {
		osTimeInstance = OSTimeSetter{
			movingTimeDifference:  0.0,
			timeToSetTime:         0.0,
			lastMovingAverageTime: time.Time{},
			lastSetTime:           time.Time{},
			c:                     make(chan setTime, 1),
			eh:					   common.NewExitHelper(),
		}
		go osTimeInstance.Run()
	})

	return &osTimeInstance
}

func (s *OSTimeSetter) SetTime(t time.Time, o time.Duration) {
	select {
	case s.c <- setTime{t, o}:
	default:
		log.Println("SetSystemTime: Queue full, disregarding value")
	}
}

func (s *OSTimeSetter) setSystemTime(gpsTime time.Time) {
	// Protect against setting weird years, this might happen during startup
	if gpsTime.Year() < 2022 {
		return
	}

	// Set OS time, other option could be using a syscall, but might not awlays work on all systems?
	setStr := gpsTime.Format("20060102 15:04:05.000") + " UTC"
	log.Printf("setting system time from %s to: '%s' difference %s\n", time.Now().Format("20060102 15:04:05.000"), setStr, time.Since(gpsTime))
	var err error
	if common.IsRunningAsRoot() {
		err = exec.Command("date", "-s", setStr).Run()
	} else {
		err = exec.Command("sudo", "date", "-s", setStr).Run()
	}
	if err != nil {
		log.Printf("Set Date failure: %s error\n", err)
	} else {
		log.Printf("Time set from GPS. Current time is %v\n", time.Now())
	}
}

// Caslibrate will measure how long it takes  to set the OS time and take this into consideration
// as additional time when calling when setting the time
func (s *OSTimeSetter) Calibrate() {
	log.Printf("Calibrating setting of OS time\n")
	beforeCalibration := time.Now()

	measure := func() float64 {
		measureTime := time.Now()
		s.setSystemTime(measureTime)
		timeItook := float64(time.Since(measureTime).Milliseconds())
		s.timeToSetTime = s.movingExpAvg(timeItook, s.timeToSetTime, float64(TIME_BETWEEN_MEASUREMENTS.Seconds()), float64(TOTAL_MEASURMENETS/5))
		return timeItook
	}

	// Take a measurement to get baseline for the filter
	s.timeToSetTime = measure()
	for i := 1; i < TOTAL_MEASURMENETS; i++ {
		took := measure()
		log.Printf("Calibrating setting of OS time Done, setting time takes %0.2fms\n", took)
		time.Sleep(TIME_BETWEEN_MEASUREMENTS)
	}
	// Settings time back to what we thank it should be
	s.setSystemTime(beforeCalibration.Add(time.Duration(s.timeToSetTime*TOTAL_MEASURMENETS*float64(TIME_BETWEEN_MEASUREMENTS.Milliseconds())) * time.Millisecond))
	log.Printf("Calibrating setting of OS time Done, setting time takes %0.2fms\n", s.timeToSetTime)
}

func (s *OSTimeSetter) movingExpAvg(value, oldValue, fdtime, ftime float64) float64 {
	alpha := 1.0 - math.Exp(-fdtime/ftime)
	r := alpha*value + (1.0-alpha)*oldValue
	return r
}

func (s *OSTimeSetter) Exit() {
	s.eh.Exit()
}

func (s *OSTimeSetter) Run() {
	s.eh.Add()
	defer s.eh.Done()

	// Function to calculate if given time if off by more than xx ms
	isOffByMoreThan := func(t time.Time, v int64) bool {
		m := time.Since(t).Milliseconds()
		return m > v || m < -v
	}

	setSystemTime := func(gpsTime setTime) {
		var maxTimeDifference int64 = 5000 //maximum time difference for averaging
		// Protect against setting weird years, this might happen during startup
		if gpsTime.t.Year() < 2022 {
			return
		}

		// We only use the moving average time difference if it's not off by some rediculous value
		if !isOffByMoreThan(gpsTime.t, maxTimeDifference) {
			s.movingTimeDifference = s.movingExpAvg(float64(time.Since(gpsTime.t).Milliseconds()), s.movingTimeDifference, float64(time.Since(s.lastMovingAverageTime).Milliseconds())/1000.0, AVERAGE_OVER)
			s.lastMovingAverageTime = time.Now()
		} else {
			s.movingTimeDifference = 0
		}

		if SHOW_TIME_DIFFERENCE_ONLY {
			if isOffByMoreThan(gpsTime.t, maxTimeDifference) {
				log.Printf("PPS Calibration mode: Based on your time source (see doc): failed, time %v of by more than %d ms\n", gpsTime.t, maxTimeDifference)
			} else {
				log.Printf("PPS Calibration mode: Based on your time source (see doc): Use GPSTimeOffsetPPS=%.0fms for your device\n", s.movingTimeDifference)
			}
		} else {
			// Set new time directly if it it's more than 300ms off
			if isOffByMoreThan(gpsTime.t, 300) {
				s.setSystemTime(gpsTime.t)
				s.lastSetTime = time.Now()
				// Otherwise use the moving average time difference and only when we are off more than ACCEPTABLE_TIME_OFFSET we set the time
			} else {
				// log.Printf("Difference %v moving average %.2fms\n", time.Since(newTime), movingTimeDifference)
				// Only try to set time if it's off by more than ACCEPTABLE_TIME_OFFSET_MS, at most once a minute
				if isOffByMoreThan(time.Now().Add(time.Duration(s.movingTimeDifference)*time.Millisecond), ACCEPTABLE_TIME_OFFSET.Milliseconds()) && time.Since(s.lastSetTime).Seconds() > 60 {
					s.setSystemTime(time.Now().UTC().Add(time.Duration(-s.movingTimeDifference+s.timeToSetTime) * time.Millisecond))
					s.lastSetTime = time.Now()
				}
			}
		}
	}

	for {
		select {
			case <-s.eh.C:
				return
			case gpsTime := <-s.c:
				setSystemTime(gpsTime)
		}
	}
}
