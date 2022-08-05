/*
	Copyright (c) 2021 R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	ais.go: Routines for reading AIS traffic
*/

package gps

import (
	"errors"
	"log"
	"sync"

	"time"

	"github.com/b3nn0/stratux/v2/common"
	cmap "github.com/orcaman/concurrent-map"
	"tinygo.org/x/bluetooth"
)

// Holds information about a device that is currently within a list of discovered devices
type discoveredDeviceInfo struct {
	Connected bool
	MAC       string
	name      string
}

// Hold's information about a device that has beeing scanned
type scanInfoResult struct {
	MAC  string
	name string
}

// Used to send from a received NMEA string data to the switchboard
type nmeaNewLine struct {
	nmeaLine string
	device   discoveredDeviceInfo // We assume that when we just received a NMEA line, the device is not lost so the data structure always exists
}

//type BleGPSDevice interface {
//	Listen(allowedDeviceList[] string, rxMessageCh chan gps.RXMessage)
//	Stop()
//}

type BleGPSDevice struct {
	adapter              bluetooth.Adapter
	discoveredDeviceList cmap.ConcurrentMap
	qh                   *common.QuitHelper
	rxMessageCh          chan<- RXMessage
	discoveredDevicesCh  chan<- DiscoveredDevice
}

func NewBleGPSDevice(rxMessageCh chan<- RXMessage, discoveredDevicesCh chan<- DiscoveredDevice) BleGPSDevice {
	return BleGPSDevice{
		adapter:              *bluetooth.DefaultAdapter,
		discoveredDeviceList: cmap.New(),
		qh:                   common.NewQuitHelper(),
		rxMessageCh:          rxMessageCh,
		discoveredDevicesCh:  discoveredDevicesCh,
	}
}

var (
	HM_10_CONF, _ = bluetooth.ParseUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
	BLE_RX, _ = bluetooth.ParseUUID("0000ffe1-0000-1000-8000-00805f9b34fb")
)

// WATCHDOG for the blue device, if we do not receive data at least once a second we will diconnect and re-connect
const WATCHDOG_RECEIVE_TIMER = 1000 * time.Millisecond

// AdvertisementListener will scan for any nearby devices add notifies them on the scanInfoResult for any found devices
func (b *BleGPSDevice) advertisementListener(scanInfoResultChan chan<- scanInfoResult) {
	b.qh.Add()
	defer func() {
		b.adapter.StopScan()
		b.qh.Done()
	}()
	executeScan := func() error {
		err := b.adapter.Scan(
			func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
				if b.qh.IsQuit() {
					b.adapter.StopScan()
				} else if result.AdvertisementPayload.HasServiceUUID(HM_10_CONF) && result.Address != nil {
					b.adapter.StopScan()
					scanInfoResultChan <- scanInfoResult{result.Address.String(), result.LocalName()}
				}
			})
		if err != nil {
			return errors.New("Error from adapter.Scan: " + err.Error())
		}
		return nil
	}

	for {
		if (b.qh.IsQuit()) {
			return
		}
		if err :=executeScan(); err!=nil {
			log.Printf("Error from ble scanner: %s", err.Error())
		}
		time.Sleep(5 * time.Second)
	}
}

/**
Coonect to our bluetooth device and listen on the RX channel for NMEA sentences
**/
func (b *BleGPSDevice) rxListener(discoveredDeviceInfo discoveredDeviceInfo, sentenceChannel chan<- nmeaNewLine, TXChannel <-chan []byte) error {
	b.qh.Add()
	defer b.qh.Done()

	address, _ := bluetooth.ParseMAC(discoveredDeviceInfo.MAC)
	btAddress := bluetooth.MACAddress{MAC: address}

	// Connect to device
	device, err := b.adapter.Connect(bluetooth.Address{MACAddress: btAddress}, bluetooth.ConnectionParams{})
	if err != nil {
		return err
	}

	log.Printf("bleGPSDevice: Connected to : %s", discoveredDeviceInfo.name)
	defer func() {
		device.Disconnect()
		log.Printf("bleGPSDevice: Disconnected from : %s", discoveredDeviceInfo.name)
	}()

	services, err := device.DiscoverServices([]bluetooth.UUID{HM_10_CONF})
	if err != nil {
		return err
	}
	service := services[0]

	// Get the two characteristics present in this service.
	chars, err := service.DiscoverCharacteristics([]bluetooth.UUID{BLE_RX})
	if err != nil {
		return err
	}

	// variables for the NMEA parser
	var receivedData []byte

	// Mutex + condition to sync the write/read routines
	mutex := sync.Mutex{}
	condition := sync.NewCond(&mutex)

	tx := chars[0]
	go func() {
		const MAX_NMEA_LENGTH = 79
		var charPosition int
		byteArray := [MAX_NMEA_LENGTH + 1]byte{} // One extra for zero termination
		sentenceStarted := false
		for {
			mutex.Lock()
			condition.Wait()
			for i := 0; i < len(receivedData); i++ {
				c := receivedData[i]
				// Within NMEA sentence?
				if sentenceStarted &&
					c >= 0x20 && c <= 0x7e &&
					charPosition < MAX_NMEA_LENGTH {
					byteArray[charPosition] = c
					charPosition++
				}

				// End of a NMEA sentence
				if c == 0x0d && sentenceStarted && charPosition < MAX_NMEA_LENGTH {
					sentenceStarted = false
					thisOne := string(byteArray[0:charPosition])
					sentenceChannel <- nmeaNewLine{thisOne, discoveredDeviceInfo}
				}

				// Start of a new NMEA sentence
				if c == '$' {
					sentenceStarted = true
					byteArray[0] = c
					charPosition = 1
				}
			}
			receivedData = receivedData[:0]
			mutex.Unlock()
		}
	}()

	// Callback from the enable notification function
	// This might (depends on underlaying implementation) run in a interrupt where we cannot allocate any heap
	// we use a mutex with signal to copy the received dataset into a existing byte array for further processing
	watchdogTimer := common.NewWatchDog(WATCHDOG_RECEIVE_TIMER)
	defer func() {
		watchdogTimer.Stop()
	}()

	// Listen to bluetooth messages
	enaNotifyErr := tx.EnableNotifications(func(value []byte) {
		// Reset the watchdig timer
		watchdogTimer.Poke()

		// Copy received data
		mutex.Lock()
		receivedData = append(receivedData, value...)
		condition.Signal()
		mutex.Unlock()
	})

	if enaNotifyErr != nil {
		return enaNotifyErr
	}

	select {
	case <-b.qh.C:
		return nil
	case <-watchdogTimer.C:
		return nil
	}
}

/**
connectionMonitor monitors the list discoveredDeviceList for disconnected devices and reconnects them again
*/
func (b *BleGPSDevice) connectionMonitor(sentenceChannel chan<- nmeaNewLine) {
	b.qh.Add()
	defer b.qh.Done()

	ticker := time.NewTicker(500 * time.Millisecond)
	for {
		select {
		case <-b.qh.C:
			return
		case <-ticker.C:
			for entry := range b.discoveredDeviceList.IterBuffered() {
				info := entry.Val.(*discoveredDeviceInfo)

				// Send a message back about if the device is connected and the name of the discovered device
				if !info.Connected {
					info.Connected = true
					go func() {
						TXChannel := make(chan []byte, 1)
						b.updateDeviceDiscoveryWithChannel(info.name, TXChannel)

						// Attempt to connect to a bluetooth device
						err := b.rxListener(*info, sentenceChannel, TXChannel)
						if err != nil {
							log.Printf("bleGPSDevice: Error from device : %s : %s", info.name, err.Error())
						} else {
							log.Printf("bleGPSDevice: Devices was finished")
						}
						info.Connected = false
						b.updateDeviceDiscovery(info.name, false)
					}()
				}
			}
		}
	}
}

func (b *BleGPSDevice) defaultDeviceDiscoveryData(name string, connected bool) DiscoveredDevice {
	return DiscoveredDevice{
		Name:               name,
		Connected:          connected,
		GpsDetectedType:    GPS_TYPE_BLUETOOTH,
		GpsSource:          GPS_SOURCE_BLUETOOTH,
		GpsTimeOffsetPpsMs: 250 * time.Millisecond, // For SoftRF t-Echo AT65 it seemed to be around 250ms
	}
}

func (b *BleGPSDevice) updateDeviceDiscovery(name string, connected bool) {
	b.discoveredDevicesCh <- b.defaultDeviceDiscoveryData(name, connected)
}

func (b *BleGPSDevice) updateDeviceDiscoveryWithChannel(name string, TXChannel chan []byte) {
	device := b.defaultDeviceDiscoveryData(name, true)
	device.TXChannel = TXChannel
	device.HasTXChannel = true
	b.discoveredDevicesCh <- device
}

func (b *BleGPSDevice) switchBoard(nmeaSentenceChannel <-chan nmeaNewLine) {
	b.qh.Add()
	defer b.qh.Done()

	for {
		select {
		case <-b.qh.C:
			return
		case nmeaSentence := <-nmeaSentenceChannel:
			select {
			case b.rxMessageCh <- RXMessage{
				Name:     nmeaSentence.device.name,
				NmeaLine: nmeaSentence.nmeaLine,
			}:
			default:
				log.Printf("BleGPSDevice: rxMessageCh Full, skipping lines")
			}
		}
	}
}

func (b *BleGPSDevice) Stop() {
	b.qh.Quit()
}

func (b *BleGPSDevice) Run(allowedDeviceList []string) {
	b.qh.Add()
	defer b.qh.Done()

	if err := b.adapter.Enable(); err != nil {
		log.Printf("bleGPSDevice: Failed to enable bluetooth adapter : %s", err.Error())
		return
	}

	scanInfoResultChannel := make(chan scanInfoResult, 1)
	nmeaSentenceChannel := make(chan nmeaNewLine, 5)

	go b.connectionMonitor(nmeaSentenceChannel)
	go b.switchBoard(nmeaSentenceChannel)
	go b.advertisementListener(scanInfoResultChannel)

	for {
		select {
		case <-b.qh.C:
			return
		case address := <-scanInfoResultChannel:
			// Only allow names we see in our list in our allowed list
			if !common.StringInSlice(address.name, allowedDeviceList) {
				// log.Printf("Device : %s %s found", address.MAC, address.Name)
				// Send a message about a discovered device, even if it's not configured for reading
				b.updateDeviceDiscovery(address.name, false)
			} else {
				added := b.discoveredDeviceList.SetIfAbsent(address.MAC, &discoveredDeviceInfo{Connected: false, MAC: address.MAC, name: address.name})
				if added {
					log.Printf("bleGPSDevice: Listen: Adding device : %s", address.name)

					// Send a message about that was added to a list
					b.updateDeviceDiscovery(address.name, false)
				}
			}
		}
	}
}
