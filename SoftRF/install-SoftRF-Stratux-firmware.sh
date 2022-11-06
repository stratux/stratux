#!/bin/bash

fwFile=SoftRF-esp32.zip
fwName=SoftRF-esp32

RED='\033[1;31m'
NC='\033[0m' # No Color

function cleanup {
  echo "Removing /tmp/Soft*"
  rm -f /tmp/Soft*
}
trap cleanup EXIT


cd "$(dirname "$0")"

echo ""
echo -e "${RED}To install the SoftRF formware you must be connected to the internet.${NC}"
echo ""
echo "To which USB would you like to install the formware at:"

list=$(ls /dev/ttyUSB*)
IFS=$'\n'
select usbDevice in $list
   do test -n "$usbDevice" && break; 
    echo "No USB device selected"
    exit
done

echo "Installing formware to $usbDevice"
echo ""

curl -L https://github.com/rvt/SoftRF/releases/latest/download/$fwFile --output /tmp/$fwFile
unzip /tmp/$fwFile -d /tmp/

sudo systemctl stop stratux

python3 esptool.py --chip esp32 --port $usbDevice --baud 921600 --before default_reset --after hard_reset \
    write_flash -u --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 \
    /tmp/$fwName.ino.bootloader.bin 0x10000 /tmp/$fwName.ino.bin 0x8000 /tmp/$fwName.ino.partitions.bin 

rm -f /tmp/Soft*

sudo systemctl start stratux
