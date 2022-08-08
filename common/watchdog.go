package common

import (
	"sync/atomic"
	"time"
)
 

type watchDog struct {
	t *time.Timer
	d time.Duration
	i uint32
	tr bool
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
			wd.tr = true
			wd.C <- struct{}{}
		}
	})

    return &wd
}

func (w *watchDog) IsTriggered() bool {
	return w.tr
}

func (w *watchDog) Poke() {
	atomic.StoreUint32(&w.i, uint32(0))
	w.t.Stop()
	w.t.Reset(w.d)
	atomic.StoreUint32(&w.i, uint32(1))
}

func (w *watchDog) Stop() {
	atomic.StoreUint32(&w.i, uint32(0))
	w.t.Stop()
}
