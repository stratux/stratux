#!/bin/bash -e
# Build and install Stratux

# install librtlsdr and librtlsdr-dev so stratux can be built
#
# NOTE: No reason to remove this as the container is discarded after the build
LIBRTLSDR_DEB="librtlsdr0_2.0.2-2_arm64.deb"
LIBRTLSDR_DEV_DEB="librtlsdr-dev_2.0.2-2_arm64.deb"

apt install -y libusb-1.0-0 libusb-1.0-0-dev
wget https://github.com/stratux/rtlsdr/releases/download/v1.0/${LIBRTLSDR_DEB}
wget https://github.com/stratux/rtlsdr/releases/download/v1.0/${LIBRTLSDR_DEV_DEB}
dpkg -i ${LIBRTLSDR_DEB}
dpkg -i ${LIBRTLSDR_DEV_DEB}

# build stratux debian packages
(cd ../../stratux && make dpkg)

# get the package name, this can vary as it contains the build version
DEB_NAME=`cd ../../stratux && ls -1t *.deb | head -1`

DEB_FILENAME_TMP=${ROOTFS_DIR}/tmp/${DEB_NAME}

# copy dpkg into /tmp directory so its available in the chroot
(cp ../../stratux/${DEB_NAME} ${DEB_FILENAME_TMP})

# install the stratux package
on_chroot << EOF
    dpkg -i /tmp/${DEB_NAME}
    dpkg -r network-manager raspberrypi-net-mods
EOF

# remove the dpkg file from tmp
rm ${DEB_FILENAME_TMP}
