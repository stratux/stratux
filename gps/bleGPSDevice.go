/*
	Copyright (c) 2021 R. van Twisk
	Distributable under the terms of The "BSD New" License
	that can be found in the LICENSE file, herein included
	as part of this header.

	ais.go: Routines for reading AIS traffic
*/

package gps

import (
	"bytes"
	"log"
	"reflect"
	"strings"
	"unsafe"

	"time"

	"github.com/b3nn0/stratux/v2/common"
	cmap "github.com/orcaman/concurrent-map/v2"
	"tinygo.org/x/bluetooth"

	"github.com/muka/go-bluetooth/bluez/profile/gatt"
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
const MAX_NMEA_LENGTH = 128
var (
	HM_10_CONF, _ = bluetooth.ParseUUID("0000ffe0-0000-1000-8000-00805f9b34fb")
	BLE_RX, _     = bluetooth.ParseUUID("0000ffe1-0000-1000-8000-00805f9b34fb")
)

// startScanningBluetoothLEDevices will scan for any nearby devices add notifies them on the scanInfoResult for any found devices
// Sometimes we get CRC errors from a attached GPS device that will look like this :GPS error. Invalid NMEA string: Checksum failed. Calculated 0X7F; expected 0X68 $GPGSV,3,1,10,01,15,256,,08,72,282,,10,50*68
func (b *BleGPSDevice) startScanningBluetoothLEDevices(leh *common.ExitHelper) {
	b.eh.Add()
	defer b.eh.Done()
	leh.Add()
	defer leh.Done()

	type scanInfoResult struct {
		MAC  string
		name string
	}

	// BlueTooth callback functions are sensetive to memory allocations so we use a scannel for it
	scanInfoCh := make(chan scanInfoResult, 5)
	
	// Start a go routine to listen for scan results
	scanner := func (done chan bool) {
		// Scan is blocking, so we need a signal to stop
		err := b.adapter.Scan(
			func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {						
				select {
				case <-done:
					adapter.StopScan()
				default:
					// Test for capability we require
					if result.AdvertisementPayload.HasServiceUUID(HM_10_CONF) && 
						result.Address != nil {

						// LocalName is the (complete or shortened) local name of the device.
						// many devices do not broadcast a local name, s
						var name = result.LocalName()
						if (strings.TrimSpace(name) == "" ) {
							name = result.Address.String()
						}

						scanInfoCh <- scanInfoResult{result.Address.String(), name}
					}
				}
			})		
		if err != nil {
			log.Printf("bleGPSDevice: Error from scanner: %s", err.Error())
		}
	}

	done := make(chan bool)
	defer close(done)
	go scanner(done)
	for {
		select {
		case <-b.eh.C:
			return
		case <-leh.C:
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
}

// This is a modified copy from the original EnableNotifications on DeviceCharacteristic
// that version did not call UnwatchProperties and this leads to when re-enabling bluetooth the callback of the origional
// callback get's called this can lead to memory leaks and undesired behaviour
// The below function does call UnwatchProperties(..) on the GattCharacteristic1 to ensure we are not keep getting called
// when we stop the BleServices
func (b *BleGPSDevice) enableNotifications(characteristic *gatt.GattCharacteristic1, callback func(buf []byte)) error {
	ch, err := characteristic.WatchProperties()
	if err != nil {
		return err
	}
	done := make(chan bool)
	go func() {
		b.eh.Add()	
		defer b.eh.Done()
		for {
			select {
			case update := <- ch:
				if update.Interface == "org.bluez.GattCharacteristic1" && update.Name == "Value" {
					callback(update.Value.([]byte))
				}
			case <- b.eh.C:
				done <- true
				return
			}
		}
	}()

	go func() {
		<- done
		characteristic.StopNotify()
		characteristic.UnwatchProperties(ch)
	}()	

	return characteristic.StartNotify()
}

/**
Coonect to our bluetooth device and listen on the RX channel for NMEA sentences
**/
func (b *BleGPSDevice) rxListener(ddi discoveredDeviceInfo) error {
	b.eh.Add()
	defer b.eh.Done()

	// Start scanner and wait for the adapter 
	ch := make(chan bluetooth.ScanResult, 1)
	var err error

	bleScanner := func (done chan bool) {
		err := b.adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
			select {
			case <-done:
				adapter.StopScan()
			default:
				if result.Address.String() == ddi.MAC {
					adapter.StopScan()
					ch <- result
				}
			}
		})
		if err != nil {
			log.Printf("bleGPSDevice: Error from scanner: %s", err.Error())
		}
	}

	done := make(chan bool)
	go bleScanner(done)
	var device *bluetooth.Device
	select {
		case result := <-ch:
			device, err = b.adapter.Connect(result.Address, bluetooth.ConnectionParams{})
			if err != nil {
				return err
			}
		// Based on https://bluetoothle.wiki/advertising
		case <-time.After(10 * time.Second):
			done <- true
			log.Printf("bleGPSDevice: Timeout looking for device " + ddi.name)
			return nil
	}
	defer device.Disconnect()

	// Find the service
	var services[] bluetooth.DeviceService
	if services, err = device.DiscoverServices([]bluetooth.UUID{HM_10_CONF}); err != nil {
		return err
	}

	// Get the two characteristics present in this service.
	var chars[] bluetooth.DeviceCharacteristic
	if chars, err = services[0].DiscoverCharacteristics([]bluetooth.UUID{BLE_RX}); err != nil {
		return err
	}

	log.Printf("bleGPSDevice: Connected to : %s", ddi.name)
	GetServiceDiscovery().Send(DiscoveredDevice{
		Name:      ddi.name,
		Content:   CONTENT_CONNECTED,
		Connected: true,
	})

	defer func() {
		GetServiceDiscovery().Connected(ddi.name, false)
	}()


	// Listen for incomming traffic
	dataChannel := make(chan []byte, 5)
	cValue := reflect.ValueOf(&chars[0]).Elem().FieldByName("characteristic")
	characteristic := reflect.NewAt(cValue.Type(), unsafe.Pointer(cValue.UnsafeAddr())).Elem().Interface().(*gatt.GattCharacteristic1)
	enaNotifyErr := b.enableNotifications(characteristic, func(value []byte) {		
		select {
		case dataChannel <- value:
		default:
			// Disabled because during shutdown this potentially can lead to a log getting filled for a few seconds during shutdown 
			// log.Printf("bleGPSDevice: Dropping data from %s", ddi.name)
		}
	})

	if enaNotifyErr != nil {
		return enaNotifyErr
	}

	bufferData := make([]byte, MAX_NMEA_LENGTH * 3)
	insertPosition := 0	// Position where we next insert the data
	endPosition := -1   // Position of last byte of valid data
	dataProcess := func(process []byte) {
		if (insertPosition + len(process)) > len(bufferData) {
			insertPosition = 0
			endPosition = -1
			return
		}

		copy(bufferData[insertPosition:], process) 
        insertPosition = insertPosition + len(process)
        endPosition = endPosition + len(process)

		for {

			// Trim to start of NMEA
			startNmea := bytes.IndexByte(bufferData, 0x24) // $
			if (startNmea==-1) {
				// If we do not have the $, there is no point in keeping the data because it will always be invalid
				// this ensures that the buffer is always empty without dirty data
				insertPosition = 0
				endPosition = -1
				break
			} else if(startNmea > 0 && startNmea < endPosition) {
				// Trim such that $ is at start
				copy(bufferData, bufferData[startNmea:])
				insertPosition = insertPosition - startNmea
				endPosition = endPosition - startNmea
			}

			// Validate if it has an end, if not we wait for more characters
			endPosn := bytes.IndexAny(bufferData, "\n\r") // \n
			if endPosn == -1 || endPosn > endPosition {
				break
			}

			nmeaString := strings.Clone(string(bufferData[0:endPosn]))
			copyLength := endPosn + 1
			copy(bufferData, bufferData[copyLength:])
			insertPosition = insertPosition - copyLength
			endPosition = endPosition - copyLength

			// Send it out
			select {
			case b.rxMessageCh <- RXMessage {
				Name:     ddi.name,
				NmeaLine: strings.TrimSpace(nmeaString),
			}:
			default:
				log.Printf("bleGPSDevice: rxMessageCh Full")
			}
		}
	}	

	watchdogTimer := common.NewWatchDog(1000 * time.Millisecond)
	for {
		select {
		case newReceived := <- dataChannel:
			watchdogTimer.Poke()
			dataProcess(newReceived)
		case <-watchdogTimer.C:
			log.Printf("bleGPSDevice: rxListener watchdog triggered")
			return nil
		case <-b.eh.C:
			return nil
		}	
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
				deviceFromList := entry.Val

				// When the device is not connected, we attemt to connect it again
				if deviceFromList.Allowed && !deviceFromList.Connected {
					deviceFromList.Connected = true
					b.discoveredDeviceList.Set(deviceFromList.MAC, deviceFromList)
					go func(thisDevice discoveredDeviceInfo) {
						// Attempt to connect to a bluetooth device
						log.Printf("bleGPSDevice: Connecting to %s / %s", thisDevice.name, thisDevice.MAC)
						err := b.rxListener(thisDevice)
						if err != nil {
							log.Printf("bleGPSDevice: Disconnected error : device:%s error=%s ", thisDevice.name, err.Error())
						} else {
							log.Printf("bleGPSDevice: Disconnected %s", thisDevice.name)
						}
						thisDevice.Connected = false
						b.discoveredDeviceList.Set(thisDevice.MAC, thisDevice)
					}(deviceFromList)
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

		if (int(m["GPSSource"].(float64)) == GPS_SOURCE_BLUETOOTH) {
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
	log.Printf("bleGPSDevice: Run: Enabled Bluetooth devices")
}

/**
* Stop the BLE Service and all go routines
*/
func (b *BleGPSDevice) Stop() {
	log.Printf("Stopping BLE service")
	b.eh.Exit()
	log.Printf("... BLE service stopped")
}