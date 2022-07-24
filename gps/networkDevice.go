package gps

import (
	"bufio"
	"log"
	"net"
	"time"

	"github.com/b3nn0/stratux/v2/common"
)

type NetworkDevice struct {
	rxMessageCh          chan<- RXMessage
	discoveredDevicesCh  chan<- DiscoveredDevice
	qh                   *common.QuitHelper
}


func NewNetworkGPSDevice(rxMessageCh chan<- RXMessage, discoveredDevicesCh chan<- DiscoveredDevice) NetworkDevice {
	m := NetworkDevice{
		rxMessageCh:          rxMessageCh,
		discoveredDevicesCh:  discoveredDevicesCh,
		qh:                   common.NewQuitHelper(),
	}
	return m
}

func (b *NetworkDevice) defaultDeviceDiscoveryData(name string, connected bool) DiscoveredDevice {
	return DiscoveredDevice{
		Name:               name,
		Connected:          connected,
		GpsDetectedType:    common.GPS_TYPE_NETWORK, // TODO: Should we be more specific for example mention that it's an SoftRF device?
		GpsSource:          common.GPS_SOURCE_NETWORK,
		GpsTimeOffsetPpsMs: 100.0 * time.Millisecond,
	}
}

func (b *NetworkDevice) updateDeviceDiscovery(name string, connected bool) {
	b.discoveredDevicesCh <- b.defaultDeviceDiscoveryData(name, connected)
}

/* Server that can be used to feed NMEA data to, e.g. to connect OGN Tracker wirelessly */
func (n *NetworkDevice) tcpNMEAInListener() {
	n.qh.Add()
	defer n.qh.Done()
	ln, err := net.Listen("tcp", ":30011")

	if err != nil {
		log.Printf(err.Error())
		return
	}

	go func() {
		<- n.qh.C
		ln.Close()
	}()

	for {
		conn, err := ln.Accept()
		if n.qh.IsQuit() {
			return
		}
		if err != nil {
			log.Printf(err.Error())
			continue
		}
		go n.handleNmeaInConnection(conn)
		time.Sleep(250 * time.Millisecond)
	}	
}

func (n *NetworkDevice) handleNmeaInConnection(c net.Conn) {
	n.qh.Add()
	defer n.qh.Done()
	reader := bufio.NewReader(c)
	remoteAddress := c.RemoteAddr().String()
	n.updateDeviceDiscovery(remoteAddress, true)

	go func() {
		<- n.qh.C
		c.Close()
	}()

	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		
		n.rxMessageCh <- RXMessage{
			Name:     remoteAddress,
			NmeaLine: line,
		}		
	}
	n.updateDeviceDiscovery(remoteAddress, false)
}

func (n *NetworkDevice) Stop() {
	n.qh.Quit()
}

func (n *NetworkDevice) Run() {
	go n.tcpNMEAInListener()
}

