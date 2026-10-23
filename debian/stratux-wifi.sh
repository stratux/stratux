#!/bin/bash
#####
#####	Wifi/AP control file
#####
#####	Description: All the scripting related to the AP and wireless functions for Stratux should be placed in this file
#####	This file is called when wlan0 is started i.e. "ifup wlan0"
#####	This script is called from /etc/network/interfaces by the line "post-up /usr/sbin/stratux-wifi.sh" under the wlan0 configuration
#####


#Logging Function
SCRIPT=`basename ${BASH_SOURCE[0]}`
STX_LOG="/var/log/stratux.log"
function wLog () {
       echo "$(date +"%Y/%m/%d %H:%M:%S")  - $SCRIPT - $1" >> ${STX_LOG}
}
wLog "Running Stratux WiFI Script."

interface=$1 # for dhcp and wpa_supplicant
mode=$2 # 0=ap, 1=wifi-direct, 2=ap+client
pin=$3 # wifi-direct pin

if [ "$1" == "0" ] || [ "$1" == "1" ] || [ "$1" == "2" ]; then
	# compatibility to old /etc/network/interfaces before eu027
        echo "COMPAT MODE"
	interface="wlan0"
	mode=$1
	pin=$2
fi

echo "interface=${interface},mode=${mode}"

function terminate {
	# Given a PID file, terminate the process specified
	if [[ -f $1 ]]; then
		pid="$(cat $1)"
		rm $1
		echo "killing $pid"
		kill $pid
		for i in $(seq 10); do
			# If process exits successfully, we are done
			echo "checking..."
			if ! ps -p $pid; then
				echo "terminated $pid"
				return
			fi
			sleep 0.5
		done
		# Didn't exit in 5 secs.. kill it
		echo "could not kill $pid. Hard kill"
		kill -9 $pid
		sleep 1
	fi

}

STRATUX_CONF="/boot/firmware/stratux.conf"
function conf-get {
	jq -r "$1 // empty" "${STRATUX_CONF}" 2>/dev/null
}

# Set the regulatory domain before the AP starts. wpa_supplicant's country= alone is not always
# applied by brcmfmac in AP mode, which can leave the radio on the restrictive world domain.
function wifi-set-regdomain {
	local country=$(conf-get .WiFiCountry)
	if [ -n "$country" ]; then
		wLog "Setting WiFi regulatory domain to $country"
		iw reg set "$country" || wLog "iw reg set $country failed"
	fi
}

# Most "EFB keeps dropping the Stratux" reports are WiFi, not Stratux. Power save on the Pi's
# brcmfmac chip adds latency spikes and lets clients time out, so always turn it off. Optionally
# pin the AP transmit power (WiFiTxPower in dBm, 0 = driver default).
function wifi-tune {
	local dev=$1
	for d in wlan0 $dev; do
		iw dev $d set power_save off 2>/dev/null && wLog "power_save off on $d"
	done
	local txpower=$(conf-get .WiFiTxPower)
	if [ -n "$txpower" ] && [ "$txpower" -gt 0 ] 2>/dev/null; then
		[ "$txpower" -gt 31 ] && txpower=31 # same bound as the Go side (networksettings.go)
		local mbm=$((txpower * 100))
		if iw dev $dev set txpower fixed $mbm || iw phy phy0 set txpower fixed $mbm; then
			wLog "TX power on $dev set to ${txpower} dBm"
		else
			wLog "Setting TX power on $dev to ${txpower} dBm failed"
		fi
	fi
}

function prepare-start {
	# Preliminaries. Kill off old services.
	wLog "Killing wpa_supplicant AP services "
	terminate /run/wpa_supplicant_ap.pid
	terminate /run/wpa_supplicant_p2p.pid

	wLog "Stopping DHCP services "
	/bin/systemctl stop dnsmasq
	/usr/bin/killall dnsmasq
}

function ap-start {
	echo "Starting AP mode on $interface"

	/sbin/wpa_supplicant -P/run/wpa_supplicant_ap.pid -B -i $interface -c /etc/wpa_supplicant/wpa_supplicant_ap.conf
	sleep 2
	wifi-tune $interface

	wLog "Restarting DHCP services"
	dnsmasq -u dnsmasq --conf-dir=/etc/dnsmasq.d -i $interface
}

function wifi-direct-start {
	echo "Starting wifi direct mode on $interface"

	/sbin/wpa_supplicant -P/run/wpa_supplicant_p2p.pid -B -i $interface -c /etc/wpa_supplicant/wpa_supplicant.conf

	wpa_cli -i $interface p2p_group_add persistent=0 freq=2
	(while wpa_cli -i p2p-wlan0-0 wps_pin any $pin > /dev/null; do sleep 1; done) & disown
	ifup p2p-wlan0-0
	wifi-tune p2p-wlan0-0

	dnsmasq -u dnsmasq --conf-dir=/etc/dnsmasq.d -i p2p-wlan0-0
}

prepare-start
wifi-set-regdomain
if [ "$mode" == "1" ]; then
	wifi-direct-start
else
	ap-start
fi
