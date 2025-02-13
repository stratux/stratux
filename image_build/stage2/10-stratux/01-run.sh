#!/bin/bash -e

# install packages as part of the run portion of the script
# as they are necessary for the build.
#
# xx-packages will install to the PI so its not appropriate for build required packages
apt update
apt install -y make gcc ncurses-dev pkg-config golang

on_chroot << EOF
    apt install --yes dnsmasq ifplugd

    # try to reduce writing to SD card as much as possible, so they don't get
    # bricked when yanking the power cable
    # Disable swap...
    systemctl disable dphys-swapfile
    apt purge -y dphys-swapfile
    apt autoremove -y
    apt clean
    #echo y | rpi-update

    systemctl enable ssh
    systemctl disable dnsmasq # we start it manually on respective interfaces
    #systemctl disable hciuart
    systemctl disable triggerhappy
    systemctl disable wpa_supplicant
    systemctl disable systemd-timesyncd # We sync time with GPS. Make sure there is no conflict if we have internet connection
    systemctl disable resize2fs_once

    systemctl disable apt-daily.timer
    systemctl disable apt-daily-upgrade.timer
    systemctl disable man-db.timer

    systemctl enable bluetooth

    # Run DHCP on eth0 when cable is plugged in
    sed -i -e 's/INTERFACES=""/INTERFACES="eth0"/g' /etc/default/ifplugd

    # Generate ssh key for all installs. Otherwise it would have to be done on each boot, which takes a couple of seconds
    ssh-keygen -A -v
    systemctl disable regenerate_ssh_host_keys
    # This is usually done by the console-setup service that takes quite long of first boot..
    /lib/console-setup/console-setup.sh

    # remove userconf-pi to remove the initial setup / configuration process gui as we
    # configure a user manually
    apt remove userconf-pi

EOF

# install esptool
on_chroot << EOF
    apt install -y python3-pip
    pip install --break-system-packages esptool
EOF

# install bluez 5.79 version shipping with current RPiOS (5.66) is buggy in peripheral mode..
BLUEZ_DEB="bluez_5.79-1_arm64.deb"
on_chroot << EOF
    cd /tmp
    wget https://github.com/stratux/bluez/releases/download/v1.0/${BLUEZ_DEB}
    dpkg -i ${BLUEZ_DEB}
    rm ${BLUEZ_DEB}
EOF

LIBRTLSDR_DEB="librtlsdr0_2.0.2-2_arm64.deb"
LIBRTLSDR_DEV_DEB="librtlsdr-dev_2.0.2-2_arm64.deb"
on_chroot << EOF
    echo "Installing librtlsdr in chroot"
    cd /tmp
    apt install -y libusb-1.0-0 libusb-1.0-0-dev
    wget https://github.com/stratux/rtlsdr/releases/download/v1.0/${LIBRTLSDR_DEB}
    wget https://github.com/stratux/rtlsdr/releases/download/v1.0/${LIBRTLSDR_DEV_DEB}
    dpkg -i ${LIBRTLSDR_DEB}
    dpkg -i ${LIBRTLSDR_DEV_DEB}
    rm ${LIBRTLSDR_DEB}
    rm ${LIBRTLSDR_DEV_DEB}

    echo "Building and installing kalibrate-rtl"

    apt install --yes build-essential autoconf libtool libfftw3-dev git

    # kalibrate-rtl
    cd /tmp
    git clone https://github.com/steve-m/kalibrate-rtl
    cd kalibrate-rtl
    ./bootstrap
    ./configure
    make -j8
    make install
    cd ../
    rm -rf kalibrate-rtl

    # remove the dev package of rtlsdr
    dpkg -r librtlsdr-dev

    # remove now unused libusb-1.0-0-dev
    apt remove -y libusb-1.0-0-dev
EOF

# Prepare wiringpi for ogn trx via GPIO
WIRINGPI_FILENAME="wiringpi_3.14_arm64.deb"
wget https://github.com/WiringPi/WiringPi/releases/download/3.14/${WIRINGPI_FILENAME}

# copy dpkg into /tmp directory so its available in the chroot
WIRINGPI_FILENAME_TMP=${ROOTFS_DIR}/tmp/${WIRINGPI_FILENAME}
(cp ${WIRINGPI_FILENAME} ${WIRINGPI_FILENAME_TMP})

# install the package
on_chroot << EOF
    dpkg -i /tmp/${WIRINGPI_FILENAME}
EOF

# remove the dpkg file from tmp
rm ${WIRINGPI_FILENAME_TMP}


install files/bashrc.txt ${ROOTFS_DIR}/root/.bashrc

install -m 644 files/motd "${ROOTFS_DIR}/etc/motd"


# network default config. TODO: can't we just implement gen_gdl90 -write_network_settings or something to generate them from template?
install files/stratux-dnsmasq.conf ${ROOTFS_DIR}/etc/dnsmasq.d/stratux-dnsmasq.conf

install files/wpa_supplicant_ap.conf ${ROOTFS_DIR}/etc/wpa_supplicant/wpa_supplicant_ap.conf
install files/interfaces ${ROOTFS_DIR}/etc/network/interfaces

# sshd config
install files/sshd_config ${ROOTFS_DIR}/etc/ssh/sshd_config

# debug aliases
install files/stxAliases.txt ${ROOTFS_DIR}/root/.stxAliases

# rtl-sdr setup
install files/rtl-sdr-blacklist.conf ${ROOTFS_DIR}/etc/modprobe.d/

# system tweaks
install files/modules.txt ${ROOTFS_DIR}/etc/modules

# boot settings
install files/config.txt ${ROOTFS_DIR}/boot/firmware/

# Create default pi password as in old times
on_chroot << EOF
    echo "pi:raspberry" | chpasswd
EOF

# rootfs overlay stuff
install files/overlayctl files/init-overlay ${ROOTFS_DIR}/sbin/

on_chroot << EOF
    overlayctl install
    # init-overlay replaces raspis initial partition size growing.. Make sure we call that manually (see init-overlay script)
    touch /var/grow_root_part
    mkdir -p /overlay/robase # prepare so we can bind-mount root even if overlay is disabled
EOF

# So we can import network settings if needed
touch ${ROOTFS_DIR}/boot/firmware/.stratux-first-boot

# startup scripts
install files/rc.local ${ROOTFS_DIR}/etc/rc.local

# Optionally mount /dev/sda1 as /var/log - for logging to USB stick
echo -e "\n/dev/sda1             /var/log        auto    defaults,nofail,noatime,x-systemd.device-timeout=1ms  0       2" >> ${ROOTFS_DIR}/etc/fstab

# disable serial console, disable rfkill state restore, enable wifi on boot
sed -i ${ROOTFS_DIR}/boot/firmware/cmdline.txt -e "s/console=serial0,[0-9]\+ /systemd.restore_state=0 rfkill.default_state=1 /"
sed -i 's/quiet//g' ${ROOTFS_DIR}/boot/firmware/cmdline.txt

# Set the keyboard layout to US.
sed -i ${ROOTFS_DIR}/etc/default/keyboard -e "/^XKBLAYOUT/s/\".*\"/\"us\"/"

# Legacy stratux.conf path so it can be found easily..
ln -s ${ROOTFS_DIR}/boot/firmware/stratux.conf ${ROOTFS_DIR}/boot/stratux.conf

# Set hostname
echo "stratux" > ${ROOTFS_DIR}/etc/hostname
sed -i ${ROOTFS_DIR}/etc/hosts -e "s/raspberrypi/stratux/g"
