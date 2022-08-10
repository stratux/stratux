package gps

import (
	"bufio"
	"fmt"
	"log"
	"net"
	"strings"
	"time"

	"github.com/b3nn0/stratux/v2/common"
)

type NetworkDevice struct {
	rxMessageCh          chan<- RXMessage
	discoveredDevicesCh  chan<- DiscoveredDevice
	eh                   *common.ExitHelper
}


func NewNetworkGPSDevice(rxMessageCh chan<- RXMessage, discoveredDevicesCh chan<- DiscoveredDevice) NetworkDevice {
	m := NetworkDevice{
		rxMessageCh:          rxMessageCh,
		discoveredDevicesCh:  discoveredDevicesCh,
		eh:                   common.NewExitHelper(),
	}
	return m
}

func (b *NetworkDevice) defaultDeviceDiscoveryData(name string, connected bool) DiscoveredDevice {
	return DiscoveredDevice{
		Name:               name,
		Connected:          connected,
		GpsDetectedType:    GPS_TYPE_NETWORK, // TODO: Should we be more specific for example mention that it's an SoftRF device?
		GpsSource:          GPS_SOURCE_NETWORK,
		GpsTimeOffsetPpsMs: 100.0 * time.Millisecond,
	}
}

func (b *NetworkDevice) updateDeviceDiscovery(name string, connected bool) {
	b.discoveredDevicesCh <- b.defaultDeviceDiscoveryData(name, connected)
}

/* Server that can be used to feed NMEA data to, e.g. to connect OGN Tracker wirelessly */
func (n *NetworkDevice) tcpNMEAInListener(port int) {
	n.eh.Add()
	defer n.eh.Done()
	log.Printf("Listening for network GPS device on port :%d\n", port)
	ln, err := net.Listen("tcp", fmt.Sprintf(":%d", port))

	if err != nil {
		log.Printf(err.Error())
		return
	}

	go func() {
		<- n.eh.C
		ln.Close()
	}()

	for {
		conn, err := ln.Accept()
		if n.eh.IsExit() {
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
	n.eh.Add()
	defer n.eh.Done()
	log.Printf("Connecting network GPS device : %s\n", c.RemoteAddr().String())

	reader := bufio.NewReader(c)
	remoteAddress := c.RemoteAddr().String()
	n.updateDeviceDiscovery(remoteAddress, true)

	go func() {
		<- n.eh.C
		c.Close()
	}()

	for {
		line, err := reader.ReadString('\r')
		if err != nil {
			break
		}
		trimedLine := strings.TrimSpace(line)
		if len(trimedLine) > 0 {
			n.rxMessageCh <- RXMessage{
				Name:     remoteAddress,
				NmeaLine: trimedLine,
			}		
		}
	}
	n.updateDeviceDiscovery(remoteAddress, false)
	log.Printf("Disconnecting network GPS device : %s\n", c.RemoteAddr().String())
}

func (n *NetworkDevice) Stop() {
	n.eh.Exit()
}

func (n *NetworkDevice) Run() {
	ports := [...]int{30011, 30012} 
	for _, port := range ports {
		go n.tcpNMEAInListener(port)
	}
}

