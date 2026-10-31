#!/bin/bash

#echo powersave >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

#Logging Function
SCRIPT=`basename ${BASH_SOURCE[0]}`
STX_LOG="/var/log/stratux.log"
function wLog () {
	echo "$(date +"%Y/%m/%d %H:%M:%S")  - $SCRIPT - $1" >> ${STX_LOG}
}
wLog "Running Stratux Updater Script."

TEMP_DIRECTORY="/boot/firmware/StratuxUpdates"

######################
# SD card protection: keep the FAT boot partition read-only while Stratux runs. Root is already
# protected by the overlay; /boot/firmware (stratux.conf, updates) is the remaining part of the card
# that gets corrupted when power is cut without shutting down. This script needs it rw for update
# handling, so make it rw now and remount it ro on exit. stratuxrun opens a short rw window itself
# whenever it saves settings or receives an update upload.
# Opt out by creating /boot/firmware/.stratux-boot-rw (e.g. from a PC).
BOOT_DIR="/boot/firmware"
function boot-lock {
	if [ -e "${BOOT_DIR}/.stratux-boot-rw" ]; then
		wLog "${BOOT_DIR}/.stratux-boot-rw present, leaving boot partition writable"
		return
	fi
	sync
	if mount -o remount,ro "${BOOT_DIR}"; then
		wLog "Boot partition remounted read-only"
	else
		wLog "WARNING: could not remount boot partition read-only"
	fi
}
if mountpoint -q "${BOOT_DIR}"; then
	mount -o remount,rw "${BOOT_DIR}" || wLog "WARNING: could not remount boot partition rw"
	trap boot-lock EXIT
fi
# Leftovers of an atomic write (writeFileAtomicFrom in stratuxrun) that was interrupted by a power cut.
rm -f "${BOOT_DIR}/stratux.conf.tmp" "${TEMP_DIRECTORY}"/*.tmp

######################
# script based update
SCRIPT_MASK="update*stratux*v*.sh"
TEMP_SCRIPT_LOCATION="$TEMP_DIRECTORY/$SCRIPT_MASK"
SCRIPT_UPDATE_LOCATION="/root/$SCRIPT_MASK"

######################
# package based update
PACKAGE_MASK="stratux*.deb"
# packages are placed here after download, before movement to PACKAGE_UPDATE_LOCATION
TEMP_PACKAGE_LOCATION="$TEMP_DIRECTORY/$PACKAGE_MASK"
# packages are placed here for update
PACKAGE_UPDATE_LOCATION="/root/$PACKAGE_MASK"

# Detect whether the overlay filesystem is currently active.
# /overlay/robase is always present (used by overlayctl for management), so we
# check the root mount type instead of directory existence.
overlay_is_active() {
	[ "$(findmnt -n -o FSTYPE /)" = "overlay" ]
}

###############
# Stage 1 (overlay active): script in SD card download location.
# Copy it directly to the lower ext4 layer so it survives the reboot.
if [ -e ${TEMP_SCRIPT_LOCATION} ]; then
	TEMP_SCRIPT_FILE=`ls -1t ${TEMP_SCRIPT_LOCATION} | head -1`
	wLog "Found update script $TEMP_SCRIPT_FILE"
	if overlay_is_active; then
		wLog "Overlay active — staging script to ext4 lower layer and disabling overlay..."
		if /sbin/overlayctl unlock && cp "${TEMP_SCRIPT_FILE}" /overlay/robase/root/; then
			chmod a+x /overlay/robase/root/$(basename "${TEMP_SCRIPT_FILE}")
			rm -f "${TEMP_SCRIPT_FILE}"
			/sbin/overlayctl disable
			wLog "Script staged. Rebooting to apply on bare ext4..."
			reboot
		else
			wLog "ERROR: Failed to stage script to ext4 lower layer. Update aborted."
			/sbin/overlayctl lock
		fi
	else
		# Overlay already inactive — copy to /root/ for next section to pick up
		cp "${TEMP_SCRIPT_FILE}" /root/
		chmod a+x /root/$(basename "${TEMP_SCRIPT_FILE}")
		rm -f "${TEMP_SCRIPT_FILE}"
	fi
fi

# Stage 2 (overlay inactive): execute the update script from /root/
if [ -e ${SCRIPT_UPDATE_LOCATION} ]; then
	UPDATE_SCRIPT_FILE=`ls -1t ${SCRIPT_UPDATE_LOCATION} | head -1`
	if [ -n "${UPDATE_SCRIPT_FILE}" ]; then
		wLog "Executing update script ${UPDATE_SCRIPT_FILE}..."
		# Move to /tmp and re-enable overlay before running, in case the script
		# triggers a service restart that kills this ExecStartPre process.
		UPDATE_TEMP_SCRIPT="/tmp/$(basename "${UPDATE_SCRIPT_FILE}")"
		mv "${UPDATE_SCRIPT_FILE}" "${UPDATE_TEMP_SCRIPT}"
		# Not 'overlayctl enable': with the overlay off it mistakes / for its lower layer
		# and remounts it read-only before the install runs.
		rm -f /overlay/disable
		if bash "${UPDATE_TEMP_SCRIPT}"; then
			wLog "Update script completed successfully."
		else
			wLog "ERROR: Update script ${UPDATE_SCRIPT_FILE} failed."
		fi
		rm -f "${UPDATE_TEMP_SCRIPT}"
		wLog "Finished. Rebooting..."
		reboot
	fi
fi

##############
# Stage 1 (overlay active): deb in SD card download location.
# Copy it directly to the lower ext4 layer so it survives the reboot,
# then disable overlay so the next boot runs dpkg on bare ext4.
if [ -e ${TEMP_PACKAGE_LOCATION} ]; then
	TEMP_PACKAGE_FILE=`ls -1t ${TEMP_PACKAGE_LOCATION} | head -1`
	wLog "Found update package $TEMP_PACKAGE_FILE"
	if overlay_is_active; then
		wLog "Overlay active — staging package to ext4 lower layer and disabling overlay..."
		if /sbin/overlayctl unlock && cp "${TEMP_PACKAGE_FILE}" /overlay/robase/root/; then
			rm -f "${TEMP_PACKAGE_FILE}"
			/sbin/overlayctl disable
			wLog "Package staged. Rebooting to install on bare ext4..."
			reboot
		else
			wLog "ERROR: Failed to stage package to ext4 lower layer. Update aborted."
			/sbin/overlayctl lock
		fi
	else
		# Overlay already inactive — copy to /root/ for next section to pick up
		cp "${TEMP_PACKAGE_FILE}" /root/
		rm -f "${TEMP_PACKAGE_FILE}"
	fi
fi

# Stage 2 (overlay inactive): install the deb from /root/ and re-enable overlay
if [ -e ${PACKAGE_UPDATE_LOCATION} ]; then
	UPDATE_PACKAGE_FILE=`ls -1t ${PACKAGE_UPDATE_LOCATION} | head -1`
	if [ -n "${UPDATE_PACKAGE_FILE}" ]; then
		wLog "Installing update package ${UPDATE_PACKAGE_FILE}..."
		# Move the deb to /tmp and re-enable overlay BEFORE calling dpkg.
		# The dpkg postinst runs 'systemctl daemon-reload && systemctl start stratux'
		# which will kill this ExecStartPre process via daemon-reload. By cleaning up
		# first the system is in a consistent state even if dpkg kills this script.
		UPDATE_TEMP_FILE="/tmp/$(basename "${UPDATE_PACKAGE_FILE}")"
		mv "${UPDATE_PACKAGE_FILE}" "${UPDATE_TEMP_FILE}"
		# Not 'overlayctl enable': with the overlay off it mistakes / for its lower layer
		# and remounts it read-only before the install runs.
		rm -f /overlay/disable
		if dpkg -i --force-depends "${UPDATE_TEMP_FILE}"; then
			wLog "Package installed successfully."
		else
			wLog "ERROR: dpkg failed to install ${UPDATE_TEMP_FILE}."
		fi
		rm -f "${UPDATE_TEMP_FILE}"
		wLog "Finished. Rebooting..."
		reboot
	fi
fi


if [ -f /boot/firmware/.stratux-first-boot ]; then
	rm /boot/firmware/.stratux-first-boot
	if [ -f /boot/firmware/stratux.conf ]; then
		# In case of US build, a stratux.conf file will always be imported, only containing UAT/OGN options.
		# We don't want to force-reboot for that.. Only for network/overlay changes
		do_reboot=false

		# re-apply overlay
		if [ "$(jq -r .PersistentLogging /boot/firmware/stratux.conf)" = "true" ]; then
			/sbin/overlayctl disable
			do_reboot=true
			wLog "overlayctl disabled due to stratux.conf settings"
		fi

		# write network config
		if grep -q WiFi /boot/firmware/stratux.conf ; then
			/opt/stratux/bin/stratuxrun -write-network-config
			do_reboot=true
			wLog "re-wrote network configuration for first-boot config import. Rebooting... Bye"
		fi
		if $do_reboot; then
			reboot
		fi
	fi
fi

wLog "Exited without updating anything..."
