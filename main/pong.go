/*
	pong.go: Jamez's Pong ADS-B monitoring and management.
	Added 1/2026
*/

package main

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strings"
	"sync"

	//"sync/atomic"
	"net"
	"os/exec"
	"time"

	// Using forked version of tarm/serial to force Linux
	// instead of posix code, allowing for higher baud rates
	"github.com/stratux/serial"
)

// pong device data
var pongSerialConfig *serial.Config
var pongSerialPort *serial.Port
var pongWG *sync.WaitGroup
var closeChpong chan int
var pongPortCloseOnce sync.Once
var pongUpdateMode bool
var pongDeviceSuccessfullyWorking bool

// closePongPort releases the serial port exactly once, regardless of whether
// the serial reader exits on its own or pongShutdown() interrupts it. The
// guarding Once is reset on each successful connect (see pongWatcher).
func closePongPort() {
	pongPortCloseOnce.Do(func() {
		if pongSerialPort != nil {
			pongSerialPort.Close()
		}
	})
}


type PongTermMessage struct {
	Text   string
	Source string
}

func initPongSerial() bool {
	var device string
	baudrate := int(3000000)

	log.Printf("Configuring Pong ADS-B\n")

	if _, err := os.Stat("/dev/pong"); err == nil {
		device = "/dev/pong"
	} else {
		log.Printf("No suitable Pong device found.\n")
		return false
	}
	log.Printf("Using %s for Pong\n", device)

	// Open port
	// No timeout specified as Pong does not heartbeat
	pongSerialConfig = &serial.Config{Name: device, Baud: baudrate}
	p, err := serial.OpenPort(pongSerialConfig)
	if err != nil {
		log.Printf("Error opening serial port: %s\n", err.Error())
		return false
	}
	errRts := p.ClearRTS()
	if errRts != nil {
		log.Printf("Clearing RTS returned %s\n", errRts.Error())
	}

	log.Printf("Pong opened serial port at %d baud\n",baudrate)

	// No device configuration is needed, we should be ready
	globalStatus.Pong_Heartbeats = 0

	pongSerialPort = p
	return true
}

func pongNetworkRepeater() {
	defer pongWG.Done()
	// Capture this session's shutdown channel locally. pongShutdown() clears the
	// global after closing it, so reading the global here would otherwise race.
	myClose := closeChpong
	log.Println("Entered Pong network repeater ...")
	cmd := exec.Command(STRATUX_HOME + "/bin/dump1090", "--net-only", "--net-stratux-port", "30006")
	stdout, _ := cmd.StdoutPipe()
	stderr, _ := cmd.StderrPipe()

	err := cmd.Start()
	if err != nil {
		log.Printf("Error executing "+STRATUX_HOME+"/bin/dump1090: %s\n", err)
		// don't return immediately, use the proper shutdown procedure
		shutdownPong = true
		<-myClose
		return
	}

	log.Println("Executed " + cmd.String() + " successfully...")

	done := make(chan bool)

	// Watch for a shutdown signal and kill dump1090 when it arrives. This runs
	// in its own goroutine so the blocking scanners below can never starve the
	// closeChpong check. (The original code only re-checked closeChpong after a
	// blocking Scan() returned, so the shutdown path was unreachable.)
	go func() {
		for {
			select {
			case <-done:
				return
			case <-myClose:
				log.Println("Pong network repeater: shutdown msg received, calling cmd.Process.Kill() ...")
				if err := cmd.Process.Kill(); err != nil {
					log.Printf("\t couldn't kill dump1090: %s\n", err)
				} else {
					log.Println("\t kill successful...")
				}
				return
			}
		}
	}()

	go func() {
		scanStdout := bufio.NewScanner(stdout)
		for scanStdout.Scan() {
			m := Dump1090TermMessage{Text: scanStdout.Text(), Source: "stdout"}
			logDump1090TermMessage(m)
		}
		if err := scanStdout.Err(); err != nil {
			log.Printf("scanStdout error: %s\n", err)
		}
	}()

	go func() {
		scanStderr := bufio.NewScanner(stderr)
		for scanStderr.Scan() {
			m := Dump1090TermMessage{Text: scanStderr.Text(), Source: "stderr"}
			logDump1090TermMessage(m)
		}
		if err := scanStderr.Err(); err != nil {
			log.Printf("scanStderr error: %s\n", err)
		}
	}()

	// Block until dump1090 exits — either on its own or because the watcher
	// goroutine above killed it — then release the watcher goroutine.
	cmd.Wait()
	close(done)
}

var dump1090ConnectionPong net.Conn = nil
var connectionErrorPong error

func pongNetworkConnection() {
	// Send to dump1090 on port 30001
	dump1090Addr := "127.0.0.1:30001"
	dump1090ConnectionPong, connectionErrorPong = net.Dial("tcp", dump1090Addr)
	// RCB monitor for connection failure and redial
}

func pongSerialReader() {
	//defer pongWG.Done()
	defer closePongPort()
	// RCB TODO channel control for terminate

	log.Printf("Starting Pong serial reader")

	scanner := bufio.NewScanner(pongSerialPort)
	for scanner.Scan() && globalStatus.Pong_connected && globalSettings.Pong_Enabled {
		pongDeviceSuccessfullyWorking = true
		s := scanner.Text()
		// Trimspace removes newlines as well as whitespace
		s = strings.TrimSpace(s)
		// Skip blank lines; indexing s[0] below would panic on an empty string.
		if len(s) == 0 {
			continue
		}
		if s[0] == '\'' {
			report := strings.Split(s, "'")
			logString :=  fmt.Sprintf("Pong ASCII: %s",report[1])
			log.Println(logString)
		} else if s[0] == '.' {
			//log.Println("Pong heartbeat\n")
			globalStatus.Pong_Heartbeats++
		} else if s[0] == '*' {
			// 1090ES report
			// Pong appends a signal strength at the end of the message
			// e.g. *8DC01C2860C37797E9732E555B23;ss=049D;
			// Remove this before forwarding to dump1090
			// We currently aren't doing anything with this information
			// and need to develop a scaling equation - we're using a
			// log detector for power so it should have a logarithmic
			// relationship. In one example, at -25dBm input (upper limit
			// of RX) we saw ~0x500. At -95dBm input (lower limit of RX)
			// we saw 0x370
			report := strings.Split(s, ";")
			//replayLog(s, MSGCLASS_DUMP1090);
			if dump1090ConnectionPong == nil {
				log.Println("Starting dump1090 network connection")
				pongNetworkConnection()
			}
			if len(report[0]) != 0 && dump1090ConnectionPong != nil {
				globalStatus.ES_messages_total++
				dump1090ConnectionPong.Write([]byte(report[0] + ";\r\n"))
				//logString := fmt.Sprintf("Relaying 1090ES: %s;", report[0])
				//log.Println(logString)
			}
		} else if s[0] == '+' || s[0] == '-' {
			// UAT report
			// Pong appends a signal strength and RS bit errors corrected
			// at the end of the message
			// e.g. -08A5DFDF3907E982585F029B00040080105C3AB4BC5C240700A206000000000000003A13C82F96C80A63191F05FCB231;rs=1;ss=A2;
			// We need to rescale the signal strength for interpretation by dump978,
			// which expects a 0-1000 base 10 (linear?) scale
			// RSSI is in hex and represents an int8 with -128 (0x80) representing an
			// errored measurement. There will be some offset from actual due to loss
			// in the path. In one example we measured 0x93 (-98) when injecting a
			// -102dBm signal
			o, msgtype := parseInput(s)
			if o != nil && msgtype != 0 {
				logString := fmt.Sprintf("Relaying UAT, type=%d: %s", msgtype,s)
				log.Println(logString)
				relayMessage(msgtype, o)
			} else if o == nil {
				log.Println("Not relaying message, o == nil")
			} else {
				log.Println("Not relaying message, msgtype == 0")
			}
		} else {
			log.Printf("Pong ASCII: %s",s)
			if strings.Contains(s,"ERROR SPI") {
				log.Printf("SPI error sent from Pong. Restarting Pong")
			}
		}
	}
	globalStatus.Pong_connected = false
	log.Printf("Exiting Pong serial reader")
	return
}

func pongShutdown() {
	log.Println("Entered Pong shutdown() ...")
	if globalStatus.Pong_connected == true {
		// Signal the network repeater (if running) to kill dump1090 and exit.
		// The repeater captured this channel locally, so clearing the global
		// here is safe and makes a second shutdown call a no-op. pongShutdown
		// only runs on the pongWatcher goroutine, so these accesses are serial.
		if closeChpong != nil {
			close(closeChpong)
			closeChpong = nil
		}
		// Interrupt the serial reader and release the port (exactly once).
		closePongPort()
		globalStatus.Pong_connected = false
	}
}

func pongSetUpdateMode() {
	pongUpdateMode = true
}

func pongRunUpdateWithOutput() error {
	/*
	out, err2 := exec.Command("/bin/bash","-c", "\"/tmp/updpong/util/updatepong.sh\"").Output()
	if err2 != nil {
		return err2
	} else {
		log.Printf("Update ran ok:\n%s\n",out)
	}
	return nil
*/
	cmd := exec.Command("/bin/bash","-c", "\"/tmp/updpong/util/updatepong.sh\"")
	stdout, _ := cmd.StdoutPipe()
	//stderr, _ := cmd.StderrPipe()

	err := cmd.Start()
	if err != nil {
		log.Printf("Error executing updatepong.sh: %s\n", err)
		return err
	}
	log.Println("Executed " + cmd.String() + " successfully...")

	scanStdout := bufio.NewScanner(stdout)
	for scanStdout.Scan() {
		m := PongTermMessage{Text: scanStdout.Text(), Source: "stdout"}
		logPongTermMessage(m)
		log.Printf("Pong Updater: %s\n",m.Text)
	}

	if err := scanStdout.Err(); err != nil {
		log.Printf("Pong Updater error: %s\n", err)
	}

	if err := cmd.Wait(); err != nil {
		return err
	} else {
		fmt.Printf("Update pong completed with success\n")
	}
	return nil
}

func pongKill() {
	// Send signal to shutdown to pongWatcher().
	shutdownPong = true
	// Spin until device has been de-initialized.
	for globalStatus.Pong_connected != false {
		time.Sleep(1 * time.Second)
	}
}

// to keep our sync primitives synchronized, only exit a read
// method's goroutine via the close flag channel check, to
// include catastrophic dongle failures
var shutdownPong bool

// Watch for config/device changes.
func pongWatcher() {
	pongDownCount := 0
	prevPongEnabled := false
	// Clear pong update mode
	pongUpdateMode = false
	pongDeviceSuccessfullyWorking = false
	for {
		time.Sleep(1 * time.Second)
		// true when a serial call fails
		if shutdownPong {
			pongShutdown()
			shutdownPong = false
			// Shutdown this reconnection loop
			break
		}
		// Autoreconnect the device
		if pongDeviceSuccessfullyWorking == true && globalSettings.Pong_Enabled && !globalStatus.Pong_connected {
			prevPongEnabled = false
		}

		if pongUpdateMode {
			log.Printf("PONG UPDATE MODE SET run the steps we need to update it here!")
			// File should be in /tmp/update_pong.zip 
			pongUpdateMode = false
			// Lets shut down the pong thread and run the update script
			pongShutdown() 
			time.Sleep(5 * time.Second)
			log.Printf("Run update process\n")
			_, err := exec.Command("/usr/bin/unzip","/tmp/update_pong.zip","-d","/tmp/updpong").Output()
			if err == nil {
				err2 := pongRunUpdateWithOutput()
				if err2 != nil {
					log.Printf("Failed to run pongRunUpdateWithOutput(): %s\n",err2.Error())
				}
			} else {
				log.Printf("Could not unpack the update file /tmp/update_pong.zip: %s\n",err.Error())
			}

			time.Sleep(1 * time.Second)
			log.Printf("update process complete - continue\n")
		}
		// Keep a counter of how long its been down
		if prevPongEnabled == false && globalSettings.Pong_Enabled == false {
			pongDownCount++
		} else {
			pongDownCount = 0
		}
		// Autodetect Pong 
		if (!globalSettings.Pong_Enabled && !prevPongEnabled && pongDownCount > 10) {
			if _, err := os.Stat("/dev/pong"); err == nil {
				log.Printf("Pong device file detected - Enabling the pong radio\n")
				globalSettings.Pong_Enabled = true
				saveSettings()
			}
		}

		if prevPongEnabled == globalSettings.Pong_Enabled {
			continue
		}

		// Global settings have changed, reconfig
		if globalSettings.Pong_Enabled && !globalStatus.Pong_connected {
			globalStatus.Pong_connected = initPongSerial()
			// This will retry next loop to connect again to the device
			if globalStatus.Pong_connected == false {
				// Relaxed polling to wait the device to be discovered
				time.Sleep(10 * time.Second)
				continue
			}
			if globalStatus.Pong_connected  {
				// Fresh shutdown channel and close-guard for this session;
				// the previous ones were consumed by the last pongShutdown().
				closeChpong = make(chan int)
				pongPortCloseOnce = sync.Once{}
				pongWG.Add(1)
				go pongNetworkRepeater()
				go pongSerialReader()
			}
			//atomic.StoreUint32(&globalStatus.Devices, uint32(count))
		} else if !globalSettings.Pong_Enabled {
			pongShutdown()
		}
		prevPongEnabled = globalSettings.Pong_Enabled
	}
}

func pongInit() {
	pongWG = &sync.WaitGroup{}
	go pongWatcher()
}

