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
	eh                   *common.ExitHelper
}

func NewNetworkGPSDevice(rxMessageCh chan<- RXMessage) NetworkDevice {
	m := NetworkDevice{
		rxMessageCh:          rxMessageCh,
		eh:                   common.NewExitHelper(),
	}
	return m
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
		} else {
			go n.handleNmeaInConnection(conn)
		}
		time.Sleep(250 * time.Millisecond)
	}	
}

func (n *NetworkDevice) handleNmeaInConnection(c net.Conn) {
	n.eh.Add()
	defer n.eh.Done()

	reader := bufio.NewReader(c)
	remoteAddress := c.RemoteAddr().String()

	GetServiceDiscovery().Send(DiscoveredDevice{
		Name:               remoteAddress,
		content:			CONTENT_TYPE | CONTENT_SOURCE | CONTENT_OFFSET_PPS | CONTENT_CONNECTED,
		Connected: 			true,
		GPSDetectedType:    GPS_TYPE_NETWORK,
		GPSSource:          GPS_SOURCE_NETWORK,
		GPSTimeOffsetPPS: 100.0 * time.Millisecond,
	})

	log.Printf("Connecting network GPS device : %s\n", c.RemoteAddr().String())

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
	GetServiceDiscovery().Connected(remoteAddress, false)
	log.Printf("Disconnecting network GPS device : %s\n", c.RemoteAddr().String())
}

func (n *NetworkDevice) Stop() {
	n.eh.Exit()
}

func (n *NetworkDevice) Run() {
	ports := [...]int{30011} 
	for _, port := range ports {
		go n.tcpNMEAInListener(port)
	}
}

