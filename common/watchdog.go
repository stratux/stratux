/*
	Copyright (c) 2022 R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.
	aprs.go: Routines for reading traffic from aprs
*/
package common

import (
	"sync/atomic"
	"time"
)
 

type watchDog struct {
	t *time.Timer
	d time.Duration
	i uint32 // THis helps to indicate that the WD is beeing poked, and not stopped
	tr uint32
	C chan struct{}
}
 
func NewWatchDog(wdTime time.Duration) *watchDog {

	wd := watchDog{
		d: wdTime,
		i: 0,
		C: make(chan struct{}),
    }

	wd.t = time.AfterFunc(wdTime, func() {
		if atomic.LoadUint32(&wd.i) != 0 {
			atomic.StoreUint32(&wd.tr, uint32(1))
			select {
			case wd.C <- struct{}{}:
			default:
			}			
		}
	})

    return &wd
}

func (w *watchDog) IsTriggered() bool {
	return atomic.LoadUint32(&w.tr) != 0
}

/**
* Poke the watchdog
*/
func (w *watchDog) Poke() {
	atomic.StoreUint32(&w.i, uint32(0))
	w.t.Stop()
	w.t.Reset(w.d)
	atomic.StoreUint32(&w.i, uint32(1))
}

/**
* Stop the WD without triggering
**/
func (w *watchDog) Stop() {
	atomic.StoreUint32(&w.i, uint32(0))
	w.t.Stop()
}

/**
* Stop the WD with triggering
**/
func (w *watchDog) Trigger() {
	atomic.StoreUint32(&w.i, uint32(1))
	w.t.Stop()
}
