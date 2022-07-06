package common
 
import (
    "sync"
)
 
type QuitHelper struct {
    C []chan struct{}
    w *sync.WaitGroup
    m *sync.Mutex
}
 
func NewQuitHelper() *QuitHelper {
    return &QuitHelper{
        w: new(sync.WaitGroup),
        m: new(sync.Mutex),
    }
}
 
func (a *QuitHelper) Add() <- chan struct{} {
    a.m.Lock()
    a.w.Add(1)
    ch := make(chan struct{})
    a.C = append(a.C, ch)
    a.m.Unlock()
    return ch
}
 
func (a *QuitHelper) Done() {
    a.w.Done()
}
 
func (a *QuitHelper) Quit() {
    a.m.Lock()
    for i := range a.C {
        go func(ii int) {
            a.C[ii] <- struct{}{}
        }(i)
    }
    a.w.Wait()
    a.C = a.C[:0]
    a.w = new(sync.WaitGroup)
    a.m.Unlock()
}
 
