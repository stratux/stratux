/*
	Copyright (c) 2021 R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	ais.go: Routines for reading AIS traffic
*/

package gps

import (
	"log"
	"strings"
	"sync"

	"time"

	"sync/atomic"

	"github.com/b3nn0/stratux/v2/common"
	cmap "github.com/orcaman/concurrent-map"
	"tinygo.org/x/bluetooth"
)

// Holds information about a device that is currently within a list of devices we connect to
type bleDeviceInfo struct {
	Connected bool
	MAC       string
	name      string
}

// Hold's information about a device that has beeing scanned
type bleScanInfo struct {
	MAC  string
	name string
}

// Used to send from a received NMEA string data to the switchboard
type nmeaNewLine struct {
	nmeaLine string
	device   bleDeviceInfo // We assume that when we just received a NMEA line, the device is not lost so the data structure always exists
}

//type BleGPSDevice interface {
//	Listen(allowedDeviceList[] string, gpsNMEALineChannel chan common.GpsNmeaLine)
//	Stop()
//}

type BleGPSDevice struct {
	adapter                 bluetooth.Adapter
	bleGPSTrafficDeviceList cmap.ConcurrentMap
	qh                      *common.QuitHelper
}

func NewBleGPSDevice() BleGPSDevice {
	return BleGPSDevice{
		adapter:                 *bluetooth.DefaultAdapter,
		bleGPSTrafficDeviceList: cmap.New(),
		qh:                      common.NewQuitHelper()}
}

/**
We must increase <limit name="max_match_rules_per_connection">8192</limit>
under /etc/dbus-1/system.d/bluetooth.conf
See logs stratux dbus-daemon[300]: [system] Connection ":1.28" is not allowed to add more match rules (increase limits in configuration file if required; max_match_rules_p
er_connection=512)
change /boot/config.txt and set # dtoverlay=disable-bt
*/

var (
	serviceUARTUUID = bluetooth.ServiceUUIDNordicUART
	rxUUID          = bluetooth.CharacteristicUUIDUARTRX
	txUUID          = bluetooth.CharacteristicUUIDUARTTX
)

const WATCHDOG_RECEIVE_TIMER = 1000 * time.Millisecond

func stringInSlice(a string, list []string) bool {
	for _, b := range list {
		if b == a {
			return true
		}
	}
	return false
}

// advertisementListener will scan for any nearby devices add notifies them on the bleScanInfoChannel for any found devices
// It's possible to get scan rounds for some time without seeing an advertisement so we should not connect to an device
// after we see it's advertisement. We rather should remember we have seen it and then connect to it at some point.
func (b BleGPSDevice) advertisementListener(allowedDeviceList []string, bleScanInfoChannel chan bleScanInfo) {
	qh := b.qh.Add() //
	defer b.qh.Done()

	// Non blocking channel so we get out of the interrupt routine quickly
	chScanResult := make(chan bluetooth.ScanResult, 1)

	go func() {
		err := b.adapter.Scan(
			// Note: within func we are within a interrupt service route
			// so we do not want to spend a lot of time here or do a lot of work
			func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
				if !result.AdvertisementPayload.HasServiceUUID(serviceUARTUUID) {
					return
				}
				// If we stop the scan we keep discovering existing devices
				// However, we get messages like these in varlog Connection ":1.10" is not allowed to add more match rules (increase limits in configuration file if required; max_match_rules_per_connection=8192)
				// If we do not top the scan, we do not correctly re-discover existing devices but we do not get the error message in the log
				chScanResult <- result
			})
		if err != nil {
			log.Printf("Failed to start scan %s", err.Error())
			return
		}
	}()

	defer func() {
		b.adapter.StopScan()
	}()

	for {
		select {
		case <-qh:
			b.adapter.StopScan()
			return
		case foundDevice := <-chScanResult:
			if foundDevice.Address == nil {
				break
			}

			// Only allow names we see in our list. It's possible that the mac changes so we match by name, but once found
			// we do assume that the MAC never changes again. However, it can change if for example a user replaces a device with the same
			// capabilities and names it the same..
			if !stringInSlice(foundDevice.LocalName(), allowedDeviceList) {
				log.Printf("Device : %s Not in approved list of %s", foundDevice.LocalName(), strings.Join(allowedDeviceList[:], ","))
				break
			}
			bleScanInfoChannel <- bleScanInfo{foundDevice.Address.String(), foundDevice.LocalName()}
			break
		}
	}
}

/**
Coonect to our bluetooth device and listen on the RX channel for NMEA sentences
**/
func (b BleGPSDevice) rxListener(bleDeviceInfo bleDeviceInfo, sentenceChannel chan nmeaNewLine) error {
	qh := b.qh.Add()
	defer b.qh.Done()

	address, _ := bluetooth.ParseMAC(bleDeviceInfo.MAC)
	btAddress := bluetooth.MACAddress{MAC: address}

	// Connect to device
	device, err := b.adapter.Connect(bluetooth.Address{MACAddress: btAddress}, bluetooth.ConnectionParams{})
	if err != nil {
		log.Printf("Failed to connect to : %s : %s", bleDeviceInfo.name, err.Error())
		return err
	}

	log.Printf("Connected to : " + bleDeviceInfo.name)
	defer func() {
		device.Disconnect()
		log.Printf("Disconnected from : %s", bleDeviceInfo.name)
	}()

	// Connected. Look up the Nordic UART Service,
	services, err := device.DiscoverServices([]bluetooth.UUID{serviceUARTUUID})
	if err != nil {
		log.Printf("Failed to discover service : %s : %s", bleDeviceInfo.name, err.Error())
		return err
	}
	service := services[0]

	// Get the two characteristics present in this service.
	chars, err := service.DiscoverCharacteristics([]bluetooth.UUID{rxUUID, txUUID})
	if err != nil {
		log.Printf("Failed RX services : %s : %s", bleDeviceInfo.name, err.Error())
		return err
	}

	// variables for the NMEA parser
	var receivedDataSize int32
	receivedData := make([]byte, 2048)
	notificationCalledca := make(chan bool)

	// Mutex + condition to sync the write/read routines
	mutex := sync.Mutex{}
	condition := sync.NewCond(&mutex)

	tx := chars[1]
	go func() {
		const MAX_NMEA_LENGTH = 79
		var charPosition int
		byteArray := [MAX_NMEA_LENGTH + 1]byte{} // One extra for zero termination
		sentenceStarted := false
		for {
			mutex.Lock()
			condition.Wait()
			size := int(atomic.LoadInt32(&receivedDataSize))
			if size > 0 {
				for i := 0; i < size; i++ {
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
						//fmt.Printf("%s\r\n", thisOne)
						sentenceChannel <- nmeaNewLine{thisOne, bleDeviceInfo}
					}

					// Start of a new NMEA sentence
					if c == '$' {
						sentenceStarted = true
						byteArray[0] = c
						charPosition = 1
					}
				}
			}
			mutex.Unlock()
		}
	}()

	// Callback from the enable notification function
	// This might (depends on underlaying implementation) run in a interrupt where we cannot allocate any heap
	// we use a mutex with signal to copy the received dataset into a existing byte array for further processing
	var exitFunction int32
	watchdogTimer := time.AfterFunc(WATCHDOG_RECEIVE_TIMER, func() {
		if exitFunction != 0 {
			notificationCalledca <- false
		}
	})
	defer func() {
		watchdogTimer.Stop()
	}()

	enaNotifyErr := tx.EnableNotifications(func(value []byte) {
		// Reset the watchdig timer
		atomic.StoreInt32(&receivedDataSize, int32(0))
		watchdogTimer.Stop()
		watchdogTimer.Reset(WATCHDOG_RECEIVE_TIMER)
		atomic.StoreInt32(&receivedDataSize, int32(1))

		// Copy received data
		mutex.Lock()
		atomic.StoreInt32(&receivedDataSize, int32(len(value)))
		copy(receivedData, value)
		condition.Signal()
		mutex.Unlock()
	})

	if enaNotifyErr != nil {
		return enaNotifyErr
	}

	defer func() {
		//tx.StopNotify()
	}()

	select {
	case <-qh:
		//tx.StopNotify()
		return nil
	case <-notificationCalledca:
		return nil
	}
}

/**
connectionMonitor monitors the list bleGPSTrafficDeviceList for disconnected devices and reconnects them again
*/
func (b BleGPSDevice) connectionMonitor(sentenceChannel chan nmeaNewLine) {
	qh := b.qh.Add()
	defer b.qh.Done()

	ticker := time.NewTicker(250 * time.Millisecond)
	for {
		select {
		case <-qh:
			return
		case <-ticker.C:
			for entry := range b.bleGPSTrafficDeviceList.IterBuffered() {
				info := entry.Val.(*bleDeviceInfo)

				if !info.Connected {
					info.Connected = true
					go func() {
						// Attempt to connect to a bluetooth device
						err := b.rxListener(*info, sentenceChannel)
						if err != nil {
							log.Printf("Error from device : %s : %s", info.name, err.Error())
						} else {
							log.Printf("Devices was finished")
						}
						info.Connected = false
					}()
				}
			}
		}
	}
}

func (b BleGPSDevice) switchBoard(nmeaSentenceChannel chan nmeaNewLine, gpsNMEALineChannel chan common.GpsNmeaLine) {
	qh := b.qh.Add()
	defer b.qh.Done()

	for {
		select {
		case <-qh:
			return
		case nmeaSentence := <-nmeaSentenceChannel:
			select {
			case gpsNMEALineChannel <- common.GpsNmeaLine{
				Name:               nmeaSentence.device.name,
				NmeaLine:           nmeaSentence.nmeaLine,
				GpsTimeOffsetPpsMs: 100.0 * time.Millisecond,
				GpsDetectedType:    common.GPS_PROTOCOL_NMEA,
				GpsSource:          common.GPS_SOURCE_BLE}:
			default:
				log.Printf("BleGPSDevice: gpsNMEALineChannel Full, skipping lines")
			}
		}

	}
}

func (b BleGPSDevice) Stop() {
	b.qh.Quit()
}

func (b BleGPSDevice) Listen(allowedDeviceList []string, gpsNMEALineChannel chan common.GpsNmeaLine) {
	qh := b.qh.Add() //
	defer b.qh.Done()

	if err := b.adapter.Enable(); err != nil {
		log.Printf("Failed to enable bluetooth adapter : %s", err.Error())
		return
	}

	newAddressChannel := make(chan bleScanInfo)
	nmeaSentenceChannel := make(chan nmeaNewLine, 0)

	go b.connectionMonitor(nmeaSentenceChannel)
	go b.switchBoard(nmeaSentenceChannel, gpsNMEALineChannel)
	go b.advertisementListener(allowedDeviceList, newAddressChannel)

	for {
		select {
		case <-qh:
			return
		case address := <-newAddressChannel:
			added := b.bleGPSTrafficDeviceList.SetIfAbsent(address.MAC, &bleDeviceInfo{Connected: false, MAC: address.MAC, name: address.name})
			if added {
				log.Printf("Adding device : %s", address.name)
			}
		}
	}
}
