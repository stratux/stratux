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
//	Listen(allowedDeviceList[] string, gpsNMEALineChannel chan common.GpsNmeaLine)
//	Stop()
//}

type BleGPSDevice struct {
	adapter                 bluetooth.Adapter
	bleGPSTrafficDeviceList cmap.ConcurrentMap
	discoveredDevices		[]string
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

// sudo apt install pi-bluetooth bluez-tools
// sudo modprobe btusb
// dtparam=krnbt=on
// dtoverlay=disable-bt
// sudo usermod -G bluetooth -a <username>

*/

var (
	serviceUARTUUID = bluetooth.ServiceUUIDNordicUART
	rxUUID          = bluetooth.CharacteristicUUIDUARTRX
	txUUID          = bluetooth.CharacteristicUUIDUARTTX
)

// WATCHDOG for the blue device, if we do not receive data at least once a second we will diconnect and re-connect
const WATCHDOG_RECEIVE_TIMER = 1000 * time.Millisecond

// AdvertisementListener will scan for any nearby devices add notifies them on the scanInfoResult for any found devices
func (b *BleGPSDevice) advertisementListener(scanInfoResultChan chan <- scanInfoResult) {
	qh := b.qh.Add()
	defer b.qh.Done()

	go func() {
		err := b.adapter.Scan(
			// Note: within func we are within a interrupt service route, no big processing allowed
			func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
				// Check for Nordic serial service
				if !result.AdvertisementPayload.HasServiceUUID(serviceUARTUUID) {
					return
				// Address must exist otherwhise we cannot connect to it
				} else if result.Address != nil {
					scanInfoResultChan <- scanInfoResult{result.Address.String(), result.LocalName()}
				}
			})
		if err != nil {
			log.Printf("Failed to start scan %s", err.Error())
			return
		}
	}()

	<-qh
	b.adapter.StopScan()
}

/**
Coonect to our bluetooth device and listen on the RX channel for NMEA sentences
**/
func (b *BleGPSDevice) rxListener(discoveredDeviceInfo discoveredDeviceInfo, sentenceChannel chan <- nmeaNewLine) error {
	qh := b.qh.Add()
	defer b.qh.Done()

	address, _ := bluetooth.ParseMAC(discoveredDeviceInfo.MAC)
	btAddress := bluetooth.MACAddress{MAC: address}

	// Connect to device
	device, err := b.adapter.Connect(bluetooth.Address{MACAddress: btAddress}, bluetooth.ConnectionParams{})
	if err != nil {
		log.Printf("Failed to connect to : %s : %s", discoveredDeviceInfo.name, err.Error())
		return err
	}

	log.Printf("Connected to : " + discoveredDeviceInfo.name)
	defer func() {
		device.Disconnect()
		log.Printf("Disconnected from : %s", discoveredDeviceInfo.name)
	}()

	// Connected. Look up the Nordic UART Service,
	services, err := device.DiscoverServices([]bluetooth.UUID{serviceUARTUUID})
	if err != nil {
		log.Printf("Failed to discover service : %s : %s", discoveredDeviceInfo.name, err.Error())
		return err
	}
	service := services[0]

	// Get the two characteristics present in this service.
	chars, err := service.DiscoverCharacteristics([]bluetooth.UUID{rxUUID, txUUID})
	if err != nil {
		log.Printf("Failed RX services : %s : %s", discoveredDeviceInfo.name, err.Error())
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
						sentenceChannel <- nmeaNewLine{thisOne, discoveredDeviceInfo}
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
		atomic.StoreInt32(&exitFunction, int32(0))
		watchdogTimer.Stop()
		watchdogTimer.Reset(WATCHDOG_RECEIVE_TIMER)
		atomic.StoreInt32(&exitFunction, int32(1))

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

	select {
	case <-qh:
		return nil
	case <-notificationCalledca:
		return nil
	}
}

/**
connectionMonitor monitors the list bleGPSTrafficDeviceList for disconnected devices and reconnects them again
*/
func (b *BleGPSDevice) connectionMonitor(sentenceChannel chan <- nmeaNewLine, discoveredDevices chan <- common.DiscoveredDevice) {
	qh := b.qh.Add()
	defer b.qh.Done()

	ticker := time.NewTicker(1000 * time.Millisecond)
	for {
		select {
		case <-qh:
			return
		case <-ticker.C:
			for entry := range b.bleGPSTrafficDeviceList.IterBuffered() {
				info := entry.Val.(*discoveredDeviceInfo)

				// Send a message back about if the device is connected and the name of the discovered device
				discoveredDevices <- common.DiscoveredDevice {
					Name: info.name,
					Connected: info.Connected,
					GpsDetectedType: common.GPS_TYPE_BLUETOOTH, // TODO: Should we be more specific for example mention that it's an SoftRF device?
					GpsSource: common.GPS_SOURCE_BLE,
				}
				
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

func (b *BleGPSDevice) switchBoard(nmeaSentenceChannel <- chan nmeaNewLine, gpsNMEALineChannel chan <- common.GpsNmeaLine) {
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
				GpsDetectedType:    common.GPS_TYPE_BLUETOOTH,
				GpsSource:          common.GPS_SOURCE_BLE}:
			default:
				log.Printf("BleGPSDevice: gpsNMEALineChannel Full, skipping lines")
			}
		}
	}
}

func (b *BleGPSDevice) Stop() {
	b.qh.Quit()
}

func (b *BleGPSDevice) Listen(allowedDeviceList []string, gpsNMEALineChannel chan <- common.GpsNmeaLine, discoveredDevices chan <- common.DiscoveredDevice) {
	qh := b.qh.Add() //
	defer b.qh.Done()

	if err := b.adapter.Enable(); err != nil {
		log.Printf("Failed to enable bluetooth adapter : %s", err.Error())
		return
	}

	scanInfoResultChannel := make(chan scanInfoResult, 1)
	nmeaSentenceChannel := make(chan nmeaNewLine, 0)

	go b.connectionMonitor(nmeaSentenceChannel, discoveredDevices)
	go b.switchBoard(nmeaSentenceChannel, gpsNMEALineChannel)
	go b.advertisementListener(scanInfoResultChannel)

	for {
		select {
		case <-qh:
			return
		case address := <-scanInfoResultChannel:
			// Only allow names we see in our list in our allowed list
			if !common.StringInSlice(address.name, allowedDeviceList) {
				log.Printf("Device : %s Not in approved list of %s", address.name, strings.Join(allowedDeviceList[:], ","))
			} else {
				added := b.bleGPSTrafficDeviceList.SetIfAbsent(address.MAC, &discoveredDeviceInfo{Connected: false, MAC: address.MAC, name: address.name})
				if added {
					log.Printf("bleGPSDevice: Listen: Adding device : %s", address.name)
				}	
			}

			// Update device discovery. I have noticed that some BLE devices do not announce anymore after beeing connected
			// So we also announce in connectionMonitor
			isConnected := false
			if b, ok := b.bleGPSTrafficDeviceList.Get(address.MAC); ok {
				isConnected = b.(*discoveredDeviceInfo).Connected
			}

			// Send a message back about if the device is connected and the name of the discovered device
			discoveredDevices <- common.DiscoveredDevice {
				Name: address.name,
				Connected: isConnected,
				GpsDetectedType: common.GPS_TYPE_BLUETOOTH, // TODO: Should we be more specific for example mention that it's an SoftRF device?
				GpsSource: common.GPS_SOURCE_BLE,
			}
		}
	}
}
