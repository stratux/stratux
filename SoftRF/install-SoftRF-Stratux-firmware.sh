#!/bin/bash

fwFile=SoftRF-esp32.zip
fwName=SoftRF-esp32

RED='\033[1;31m'
NC='\033[0m' # No Color

function cleanup {
  echo "Removing /tmp/Soft*"
  rm -f /tmp/Soft*
  sudo systemctl start stratux
}
trap cleanup EXIT


cd "$(dirname "$0")"

echo ""
echo "To which USB would you like to install the formware at?"
echo ""
echo -e "${RED}If you are unsure about which USB device is which (sorry we cannot detect..) then unplug${NC}"
echo -e "${RED}the device you do not want to flash and re-run this script.${NC}"
echo ""
list=$(find /dev/ -type c -regextype egrep -regex '\/dev\/(ttyACM|ttyAMA|ttyUSB)[0-9]')

if [ -z "$list" ]; then
    echo "No connected USB, ACM or AMA device found"
    exit
fi

select usbDevice in $list
   do test -n "$usbDevice" && break; 
    echo "No USB device selected"
    exit
done

echo "Installing firmware to $usbDevice"
echo ""

unzip $fwFile -d /tmp/
systemctl stop stratux
echo ""

python3 esptool.py --chip esp32 --port $usbDevice --baud 921600 --before default_reset --after hard_reset \
    write_flash -u --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 \
    /tmp/$fwName.ino.bootloader.bin 0x10000 /tmp/$fwName.ino.bin 0x8000 /tmp/$fwName.ino.partitions.bin 
