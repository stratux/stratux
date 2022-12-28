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
	"strings"
	"sync"

	"time"

	"github.com/b3nn0/stratux/v2/common"
	cmap "github.com/orcaman/concurrent-map/v2"
	"tinygo.org/x/bluetooth"
)

// Holds information about a device that is currently within a list of discovered devices
type discoveredDeviceInfo struct {
	Connected bool		// Indicate if this device is current connected and communicating
	MAC       string	// MAC aadress of this device
	name      string	// Name of this device, taken from LocalName or MAC if not available
	Allowed   bool		// Indicate if this device is allowed to be used as a GPS source
}


type BleGPSDevice struct {
	adapter              bluetooth.Adapter
	discoveredDeviceList cmap.ConcurrentMap[string, discoveredDeviceInfo]
	eh                   *common.ExitHelper
	rxMessageCh          chan<- RXMessage
}

func NewBleGPSDevice(rxMessageCh chan<- RXMessage) BleGPSDevice {
	return BleGPSDevice{
		adapter:              *bluetooth.DefaultAdapter,
		discoveredDeviceList: cmap.New[discoveredDeviceInfo](),
		eh:                   common.NewExitHelper(),
		rxMessageCh:          rxMessageCh,
	}
}

const STARTUP_BLE_SCAN_TIME = 300	// Time to search for BLE devices after
var (
	HM_10_CONF, _ = bluetooth.ParseUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
	BLE_RX, _     = bluetooth.ParseUUID("0000ffe1-0000-1000-8000-00805f9b34fb")
)

// startScanningBluetoothLEDevices will scan for any nearby devices add notifies them on the scanInfoResult for any found devices
// Sometimes we get CRC errors from a attached GPS device that will look like this :GPS error. Invalid NMEA string: Checksum failed. Calculated 0X7F; expected 0X68 $GPGSV,3,1,10,01,15,256,,08,72,282,,10,50*68
func (b *BleGPSDevice) startScanningBluetoothLEDevices(leh *common.ExitHelper) {
	leh.Add()
	defer leh.Done()
	b.eh.Add()
	defer b.eh.Done()
	defer b.adapter.StopScan()

	type scanInfoResult struct {
		MAC  string
		name string
	}

	// These BlueTooth callback functions are sensetive to memry allocations so we use a scannel for it
	scanInfoCh := make(chan scanInfoResult, 5)

	// Start a go routine to listen for scan results
	go func() {
		for {
			select {
			case <-leh.C:
				return
			case <-b.eh.C:
				return
			case address := <- scanInfoCh:
				// Only allow names we see in our list in our allowed list
				added := b.discoveredDeviceList.SetIfAbsent(address.MAC, discoveredDeviceInfo{Connected: false, Allowed: false, MAC: address.MAC, name: address.name})
				if added {
					log.Printf("bleGPSDevice: Device %s found", address.name)
	
					GetServiceDiscovery().Send(DiscoveredDevice{
						Name:             address.name,
						MAC:			  address.MAC,
						Content:          CONTENT_TYPE | CONTENT_SOURCE | CONTENT_OFFSET_PPS,
						GPSDetectedType:  GPS_TYPE_BLUETOOTH,
						GPSSource:        GPS_SOURCE_BLUETOOTH,
						GPSTimeOffsetPPS: 200 * time.Millisecond,
					})
				}
			}
		}
	}()

	// Scan is blocking, keep that in mind so we do not exit this function
	err := b.adapter.Scan(
		func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
			// This test for exit needed otherwise if there is no bluetooth device at all, we keep scanning forever
			if result.AdvertisementPayload.HasServiceUUID(HM_10_CONF) && 
				result.Address != nil {

				// LocalName is the (complete or shortened) local name of the device.
				// Please note that many devices do not broadcast a local name, but may
				// broadcast other data (e.g. manufacturer data or service UUIDs) with which
				// they may be identified.
				var name = result.LocalName()
				if (strings.TrimSpace(result.LocalName()) == "" ) {
					name = result.Address.String()
				}

				scanInfoCh <- scanInfoResult{result.Address.String(), name}
			}
		})		

	if err != nil {
		log.Printf("bleGPSDevice: Error from scanner: %s", err.Error())
	}
}

/**
Coonect to our bluetooth device and listen on the RX channel for NMEA sentences
**/
func (b *BleGPSDevice) rxListener(ddi discoveredDeviceInfo) error {
	b.eh.Add()
	defer b.eh.Done()

	address, _ := bluetooth.ParseMAC(ddi.MAC)
	btAddress := bluetooth.MACAddress{MAC: address}

	// Connect to device
	// bluetooth.ConnectionParams{} is not even used in the Connect function
	device, err := b.adapter.Connect(bluetooth.Address{MACAddress: btAddress}, bluetooth.ConnectionParams{})
	if err != nil {
		return err
	}
	defer device.Disconnect()

	// Find the service
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

	log.Printf("bleGPSDevice: Connected to : %s", ddi.name)

	GetServiceDiscovery().Send(DiscoveredDevice{
		Name:      ddi.name,
		Content:   CONTENT_CONNECTED,
		Connected: true,
	})
	defer GetServiceDiscovery().Connected(ddi.name, false)

	// Mutex + condition to sync the write/read routines
	mutex := sync.Mutex{}
	condition := sync.NewCond(&mutex)
	defer condition.Signal() // This to give the below go routine a chance to exit when it's waiting for the condition

	// Callback from the enable notification function
	// This might (depends on underlaying implementation) run in a interrupt where we cannot allocate any heap
	// we use a mutex with signal to copy the received dataset into a existing byte array for further processing
	watchdogTimer := common.NewWatchDog(1000 * time.Millisecond)
	defer watchdogTimer.Stop()

	// variables for the NMEA parser
	var receivedData []byte

	tx := chars[0]
	go func() {
		const MAX_NMEA_LENGTH = 79
		var charPosition int
		byteArray := [MAX_NMEA_LENGTH + 1]byte{} // One extra for zero termination
		sentenceStarted := false
		for {
			mutex.Lock()
			condition.Wait()
			if watchdogTimer.IsTriggered() {
				log.Printf("bleGPSDevice: watchdogTimer triggered, exiting")
				return
			}
			for i := 0; i < len(receivedData); i++ {
				c := receivedData[i]
				// Within NMEA sentence?
				if sentenceStarted &&
					c >= 0x20 && c <= 0x7e && // NMEA Characters that we expect
					charPosition < MAX_NMEA_LENGTH {
					byteArray[charPosition] = c
					charPosition++
				}

				// End of a NMEA sentence
				if c == 0x0d && sentenceStarted && charPosition < MAX_NMEA_LENGTH {
					sentenceStarted = false
					thisOne := strings.Clone(string(byteArray[0:charPosition]))
					select {
					case b.rxMessageCh <- RXMessage{
						Name:     ddi.name,
						NmeaLine: thisOne,
					}:
					default:
						log.Printf("bleGPSDevice: rxMessageCh Full")
					}
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

	// Listen for incomming traffic
	enaNotifyErr := tx.EnableNotifications(func(value []byte) {
		// Reset the watchdog
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
	case <-b.eh.C:
		return nil
	case <-watchdogTimer.C:
		return errors.New("Watchdog timed out")
	}
}

/**
connectionMonitor monitors the list discoveredDeviceList for disconnected devices and reconnects them again
*/
func (b *BleGPSDevice) connectionMonitor() {
	b.eh.Add()
	defer b.eh.Done()

	ticker := time.NewTicker(15 * time.Second)
	for {
		select {
		case <-b.eh.C:
			return
		case <-ticker.C:
			for entry := range b.discoveredDeviceList.IterBuffered() {
				info := entry.Val

				// When the device is not connected, we attemt to connect it again
				if info.Allowed {

					if !info.Connected {
						info.Connected = true
						b.discoveredDeviceList.Set(entry.Key, info)
						go func(deviceToStart discoveredDeviceInfo) {
							// Attempt to connect to a bluetooth device
							log.Printf("bleGPSDevice: connection attempt %s", deviceToStart.name)
							err := b.rxListener(deviceToStart)
							if err != nil {
								log.Printf("bleGPSDevice: Device error : device:%s error=%s ", deviceToStart.name, err.Error())
							} else {
								log.Printf("bleGPSDevice: Device was finished %s", deviceToStart.name)
							}
							deviceToStart.Connected = false
							b.discoveredDeviceList.Set(deviceToStart.MAC, deviceToStart)
						}(info)
					}
				}
			}
		}
	}
}

// Sets the initial configuration for the BLE device from the config file
// All devices in this list are always allowed to connect (they came from stratux.conf)
// And they will set the CInnection to false. Assumed is that no device is running at startup
func (b *BleGPSDevice) setInitialConfiguration( data map[string]interface{}) {
	// loop over map to create discoery
	for key, value := range data {
		m := value.(map[string]interface{})
 		b.discoveredDeviceList.Set(m["MAC"].(string), discoveredDeviceInfo{
			Connected: false,
			MAC: m["MAC"].(string), 
			Allowed: true,
			name: key})

			// Send out a discovery so the GPS subsystem know's about this device
			GetServiceDiscovery().Send(DiscoveredDevice{
				Name:             key,
				Content:          CONTENT_TYPE | CONTENT_SOURCE | CONTENT_OFFSET_PPS,
				GPSDetectedType:  GPS_TYPE_BLUETOOTH,
				GPSSource:        GPS_SOURCE_BLUETOOTH,
				GPSTimeOffsetPPS: 200 * time.Millisecond,
			})			
	}
}

/**
Retreive the current configuration that can be stored in the config file
*/
func (b *BleGPSDevice) GetConfig( ) map[string]interface{} {
	data := make(map[string]interface{})
	for entry := range b.discoveredDeviceList.IterBuffered() {
		if (entry.Val.Allowed) {
			data[entry.Val.MAC] = entry.Val
		}
	}
	return data
}

/**
Put the Bluetooth controller in scanning mode and scan for comaptible devices
*/
func (b *BleGPSDevice) Scan(leh *common.ExitHelper) {	
	log.Printf("bleGPSDevice: Start scanning Bluetooth LE devices")
	b.startScanningBluetoothLEDevices(leh)
	log.Printf("bleGPSDevice: Stop scanning Bluetooth LE devices")
}

/**
* Start the BLE service
*/
func (b *BleGPSDevice) Run(deviceList map[string]interface{}) {
	if err := b.adapter.Enable(); err != nil {
		log.Printf("bleGPSDevice: Failed to enable bluetooth LE adapter : %s", err.Error())
		return
	}
	b.setInitialConfiguration(deviceList)
	go b.connectionMonitor()
	// Scan for any bluetooth for 5 minutes. We do this because when the BT did not 'see'any devices, it's impossible to connect
	go func() {
		b.eh.Add()
		defer b.eh.Done()
		log.Printf("bleGPSDevice: Scan BLE on startup for %d seconds", STARTUP_BLE_SCAN_TIME)
		leh := common.NewExitHelper();
		defer leh.Exit()
		go b.startScanningBluetoothLEDevices(leh)
		scanTime := time.NewTimer(STARTUP_BLE_SCAN_TIME * time.Second)
		select {
		case <- scanTime.C:
			break
		case <- b.eh.C:
			break
		}
		log.Printf("bleGPSDevice: Stop Scan BLE on startup")
	}()
}

/**
* Stop the BLE Service and all go routines
*/
func (b *BleGPSDevice) Stop() {
	b.eh.Exit()
}