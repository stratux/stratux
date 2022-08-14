/*
 	Copyright (c) 2022 R. van Twisk
 	Distributable under the terms of The "BSD New" License
 	that can be found in the LICENSE file, herein included
 	as part of this header.
 	aprs.go: Routines for reading traffic from aprs
 */
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
const CALIBRATION = "Calibration"
type OSTimeSetter struct {
	movingTimeDifference  float64   // Keeps track on the difference between GPS time and OS time in seconds
	timeToSetTime         float64   // Time it takes in seconds to set the system time from a given time, this is due to starting of a process in the OS
	lastMovingAverageTime time.Time // Last time when moving avergae time was set
	lastSetTime           time.Time // Last time when the time was set
	c                     chan setTime
	eh                   *common.ExitHelper
}

type setTime struct {
	t time.Time		// Time to set the system time to
	o time.Duration	// Offset to add to the time
	i string		// Issuer, user for logging
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
			c:                     make(chan setTime),
			eh:					   common.NewExitHelper(),
		}
		go osTimeInstance.Run()
	})

	return &osTimeInstance
}

func (s *OSTimeSetter) SetTime(t time.Time, o time.Duration, i string) {
	select {
	case s.c <- setTime{t, o, i}:
	default:
		 // log.Println("SetSystemTime: Queue full, disregarding value")
	}
}

func (s *OSTimeSetter) setSystemTime(gpsTime time.Time, o time.Duration, i string) {
	// Protect against setting weird years, this might happen during startup
	if gpsTime.Year() < 2022 {
		return
	}

	// Set OS time, other option could be using a syscall, but might not awlays work on all systems?
	setStr := gpsTime.Add(o).Format("20060102 15:04:05.000") + " UTC"
	if (i != "Calibration") { // Just so we do not full ip the log
		log.Printf("%s is changing system time from %s to: '%s' difference %s\n", i, time.Now().UTC().Format("20060102 15:04:05.000"), gpsTime.UTC().Format("20060102 15:04:05.000"), time.Since(gpsTime))
	}
	var err error
	if common.IsRunningAsRoot() {
		err = exec.Command("date", "-s", setStr).Run()
	} else {
		err = exec.Command("sudo", "date", "-s", setStr).Run()
	}
	if err != nil {
		log.Printf("Set Date failure: %s error\n", err)
	}
}

// Caslibrate will measure how long it takes to set the OS time and take this into consideration
// It will try to set time back as close as possible to what it thinks it should be
func (s *OSTimeSetter) Calibrate() {
	log.Printf("%s setting of OS time\n", CALIBRATION)

	measure := func() float64 {
		measureTime := time.Now()
		s.setSystemTime(measureTime, time.Duration(0), CALIBRATION)
		timeItook := float64(time.Since(measureTime).Seconds())
		s.timeToSetTime = s.movingExpAvg(timeItook, s.timeToSetTime, float64(TIME_BETWEEN_MEASUREMENTS.Seconds()), 5.0)
		return timeItook
	}

	// Take a measurement to get baseline for the filter, first a call to get base measurement somewhat correct
	beforeCalibration := time.Now()
	measure() // Do a dummy setting to set the cache
	s.timeToSetTime = measure() // Do a measurement to set a good start value for timeToSetTime
	for i := 0; i < TOTAL_MEASURMENETS; i++ {
		took := measure()
		log.Printf("%s setting time for test %d takes %0.2fms\n", CALIBRATION, i, took * 1000.0)
		time.Sleep(TIME_BETWEEN_MEASUREMENTS)
	}
	// Settings time back to what we think it should be, total number of measurements + 2 extra for setteling the system
	totalTimeTHisSHouldHavetakenS := (s.timeToSetTime+float64(TIME_BETWEEN_MEASUREMENTS.Seconds())) * TOTAL_MEASURMENETS + s.timeToSetTime*2
	s.setSystemTime(beforeCalibration.Add(time.Duration(totalTimeTHisSHouldHavetakenS) * time.Second), time.Duration(0), CALIBRATION + "_")
	log.Printf("%s setting of OS time Done, setting time takes %0.2fms\n", CALIBRATION, s.timeToSetTime * 1000.0)
}

// Moving average filter to smooth out the time difference
// value is the new value to add to the moving average
// oldValue is the old value of the moving average
// fdTime is the time between measurements
// ftTime is the time to average over
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
		return m > v || -m > v
	}

	processNewTime := func(gpsTime setTime) {
		const MAX_TIME_DIFFERENCE int64 = 5000 //maximum time difference for averaging
		withPPSGPSTime := gpsTime.t.Add(gpsTime.o) // withPPSGPSTime is the time correct with PPS
		// log.Printf("%s requests OS time %v with PPS %v", gpsTime.i, gpsTime.t, gpsTime.o)

		// Protect against setting weird years, this might happen during startup
		if withPPSGPSTime.Year() < 2022 {
			return
		}

		// We only use the moving average time difference if it's not off by some rediculous value
		if !isOffByMoreThan(withPPSGPSTime, MAX_TIME_DIFFERENCE) {
			if SHOW_TIME_DIFFERENCE_ONLY {
				s.movingTimeDifference = s.movingExpAvg(time.Since(gpsTime.t).Seconds(), s.movingTimeDifference, float64(time.Since(s.lastMovingAverageTime).Seconds()), AVERAGE_OVER)
			} else {
				s.movingTimeDifference = s.movingExpAvg(time.Since(withPPSGPSTime).Seconds(), s.movingTimeDifference, float64(time.Since(s.lastMovingAverageTime).Seconds()), AVERAGE_OVER)
			}
			s.lastMovingAverageTime = time.Now()
		} else {
			s.movingTimeDifference = 0
		}

		if SHOW_TIME_DIFFERENCE_ONLY {
			if (time.Since(s.lastSetTime).Seconds() >  5) {
				s.lastSetTime = time.Now()
				if isOffByMoreThan(withPPSGPSTime, MAX_TIME_DIFFERENCE) {
					log.Printf("PPS Calibration mode: Based on your time source (see doc): failed, time %v of by more than %d ms\n", withPPSGPSTime, MAX_TIME_DIFFERENCE)
				} else {
					log.Printf("PPS Calibration mode: Based on your time source (see doc): Use GPSTimeOffsetPPS=%.0fms for your device\n", s.movingTimeDifference * 1000.0)
				}
			}
		} else {
			if (time.Since(s.lastSetTime).Seconds() >  5) {
				// Set new time directly if it it's more than 300ms off
				if isOffByMoreThan(withPPSGPSTime, 300) {
					s.setSystemTime(withPPSGPSTime, time.Duration(s.timeToSetTime) * time.Second, gpsTime.i)
					s.lastSetTime = time.Now()
				} else {
					// log.Printf("Difference %v moving average %.2fms\n", time.Since(withPPSGPSTime), s.movingTimeDifference * 1000.0)
					// Only try to set time if it's off by more than ACCEPTABLE_TIME_OFFSET, at most once a minute
					if isOffByMoreThan(withPPSGPSTime, int64(ACCEPTABLE_TIME_OFFSET)) && time.Since(s.lastSetTime).Seconds() > 60 {
						s.setSystemTime(withPPSGPSTime.Add(time.Duration(-s.movingTimeDifference) * time.Second), time.Duration(s.timeToSetTime) * time.Second, gpsTime.i)
						s.lastSetTime = time.Now()
					}
				}
			}
		}
	}

	for {
		select {
			case <-s.eh.C:
				return
			case gpsTime := <-s.c:
				processNewTime(gpsTime)
		}
	}
}
