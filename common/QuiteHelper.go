package common
 
import (
    "sync"
    "github.com/tevino/abool/v2"
)
 
type QuitHelper struct {
    c chan struct{}
    w *sync.WaitGroup
    m sync.Mutex
    b *abool.AtomicBool
}
 
func NewQuitHelper() *QuitHelper {
    return &QuitHelper{
        c: make(chan struct{}),
        w: new(sync.WaitGroup),
        m: sync.Mutex{},
        b: abool.New(),
    }
}
 
func (a *QuitHelper) Add() <- chan struct{} {
    a.m.Lock()
    a.w.Add(1)
    a.m.Unlock()
    return a.c
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
    close(a.c)
    a.w.Wait()
    a.c = make(chan struct{})
    a.w = new(sync.WaitGroup)
    a.b.UnSet()
    a.m.Unlock()
}
 
