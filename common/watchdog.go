package common

import (
	"time"
	"sync/atomic"
)
 

type watchDog struct {
	w *time.Timer
	t time.Duration
	i uint32
	C chan struct{}
}
 
func NewWatchDog(wdTime time.Duration) *watchDog {

	wd := watchDog{
		t: wdTime,
		i: 0,
		C: make(chan struct{}),
    }

	wd.w = time.AfterFunc(wdTime, func() {
		if atomic.LoadUint32(&wd.i) != 0 {
			wd.C <- struct{}{}
		}
	})

    return &wd
}

func (w *watchDog) Take() {
	atomic.StoreUint32(&w.i, uint32(0))
	w.w.Stop()
	w.w.Reset(w.t)
	atomic.StoreUint32(&w.i, uint32(1))
}

func (w *watchDog) Stop() {
	atomic.StoreUint32(&w.i, uint32(0))
	w.w.Stop()
}
