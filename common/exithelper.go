/*
	Copyright (c) 2022 R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.
	aprs.go: Routines for reading traffic from aprs
*/
package common

import (
	"sync"

	"github.com/tevino/abool/v2"
)

/**
* ExitHelper is a small helper to make sure that one or more go routines are exited correctly
* by signalling them to exit and waiting for them to exit
*/
type ExitHelper struct {
    C chan struct{}
    w *sync.WaitGroup
    m sync.Mutex
    b *abool.AtomicBool
}
 
/**
* Create a new ExitHelper
*/
func NewExitHelper() *ExitHelper {
    return &ExitHelper{
        C: make(chan struct{}),
        w: new(sync.WaitGroup),
        m: sync.Mutex{},
        b: abool.New(),
    }
}
 
/**
* Tell the exitHelper that that there is a new function that needs to be called
* to be notified of exits
*/
func (a *ExitHelper) Add() {
    a.m.Lock()
    a.w.Add(1)
    a.m.Unlock()
}
 
/**
* Call this function when you are done with your function
* only call Done() once and only when you have called Add() Once
*/
func (a *ExitHelper) Done() {
    a.w.Done()
}

/** 
* IsExit will return true if we are actually Exiting. 
* After the Exit was complete, this function will return false again
*/
func (a *ExitHelper) IsExit() bool {
    return a.b.IsSet()
}

/**
* Call this function to exit any go routimes that needs to be exited
* and are listing to events or waiting for the IsExit() function to return true
*/
func (a *ExitHelper) Exit() {
    a.m.Lock()
    a.b.Set()
    close(a.C)
    a.w.Wait()
    a.C = make(chan struct{})
    a.w = new(sync.WaitGroup)
    a.b.UnSet()
    a.m.Unlock()
}
 
