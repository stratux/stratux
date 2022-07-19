package common
 
import (
    "sync"
    "github.com/tevino/abool/v2"
)
 
type QuitHelper struct {
    C chan struct{}
    w *sync.WaitGroup
    m sync.Mutex
    b *abool.AtomicBool
}
 
func NewQuitHelper() *QuitHelper {
    return &QuitHelper{
        C: make(chan struct{}),
        w: new(sync.WaitGroup),
        m: sync.Mutex{},
        b: abool.New(),
    }
}
 
func (a *QuitHelper) Add() {
    a.m.Lock()
    a.w.Add(1)
    a.m.Unlock()
}
 
func (a *QuitHelper) Done() {
    a.w.Done()
}

func (a *QuitHelper) IsQuit() bool {
    return a.b.IsSet()
}

func (a *QuitHelper) Quit() {
    a.m.Lock()
    a.b.Set()
    close(a.C)
    a.w.Wait()
    a.C = make(chan struct{})
    a.w = new(sync.WaitGroup)
    a.b.UnSet()
    a.m.Unlock()
}
 
