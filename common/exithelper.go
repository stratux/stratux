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
 
type ExitHelper struct {
    C chan struct{}
    w *sync.WaitGroup
    m sync.Mutex
    b *abool.AtomicBool
}
 
func NewExitHelper() *ExitHelper {
    return &ExitHelper{
        C: make(chan struct{}),
        w: new(sync.WaitGroup),
        m: sync.Mutex{},
        b: abool.New(),
    }
}
 
func (a *ExitHelper) Add() {
    a.m.Lock()
    a.w.Add(1)
    a.m.Unlock()
}
 
func (a *ExitHelper) Done() {
    a.w.Done()
}

func (a *ExitHelper) IsExit() bool {
    return a.b.IsSet()
}

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
 
