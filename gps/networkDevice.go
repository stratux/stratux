/*
 	Copyright (c) 2022 R. van Twisk
 	Distributable under the terms of The "BSD New" License
 	that can be found in the LICENSE file, herein included
 	as part of this header.
 	aprs.go: Routines for reading traffic from aprs
 */
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
	}	
}

func (n *NetworkDevice) handleNmeaInConnection(c net.Conn) {
	n.eh.Add()
	defer n.eh.Done()

	go func() {
		<- n.eh.C
		c.Close()
	}()

	remoteAddress := c.RemoteAddr().String()
	log.Printf("Connecting network GPS device : %s\n", remoteAddress)

	GetServiceDiscovery().Send(DiscoveredDevice{
		Name:               remoteAddress,
		Content:			CONTENT_TYPE | CONTENT_SOURCE | CONTENT_OFFSET_PPS | CONTENT_CONNECTED,
		Connected: 			true,
		GPSDetectedType:    GPS_TYPE_NETWORK,
		GPSSource:          GPS_SOURCE_NETWORK,
		GPSTimeOffsetPPS:   100.0 * time.Millisecond,
	})

	reader := bufio.NewReader(c)
	for {
		line, err := reader.ReadString('\r')
		if err != nil {
			break
		}
		if line = strings.TrimSpace(line); len(line) > 0 {
			select {
			case n.rxMessageCh <- RXMessage{
				Name:     remoteAddress,
				NmeaLine: line,
			}:
			default:
				log.Printf("Network rxMessageCh Full")
			}
		}
	}
	GetServiceDiscovery().Connected(remoteAddress, false)
	log.Printf("Disconnecting network GPS device : %s\n", remoteAddress)
}

func (n *NetworkDevice) Stop() {
	log.Printf("Stopping Network service")
	n.eh.Exit()
	log.Printf("... Network service stopped")
}

func (n *NetworkDevice) Scan(leh *common.ExitHelper) {
	// Not implemented
}

func (n *NetworkDevice) Run() {
	ports := [...]int{30011} 
	for _, port := range ports {
		go n.tcpNMEAInListener(port)
	}
	log.Printf("networkDevice: Run: Enabled Network devices")
}

