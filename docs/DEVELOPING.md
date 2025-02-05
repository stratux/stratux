# Development

## Building

NOTE: Stratux uses submodules, ensure you have run:

```
git submodule update --init --recursive
```

Prior to building with:

```
make
```

## OTA upgrade process

There are two mechanisms used for OTA updates in Stratux.

1. Dpkg (Debian .deb package) (<font style='background: purple'>DEB</font>)
   * Used for Stratux application updates.
1. Update script (<font style='background: green'>US</font>)
   * Was used for Stratux application updates until 2025-02-04 and remains available to perform system related update operations that are outside of the Stratux application itself, and shouldn't be included in the (<font style='background: purple'>DEB</font>) package.

Both of the update processes are similar and run through the same code paths.

### OTA update process

1. Update dpkg file(<font style='background: purple'>DEB</font>)  or (<font style='background: green'>US</font>) is uploaded via the Stratux web interface (settings.js)
1. The <font style='background: purple'>DEB</font> / <font style='background: green'>US</font> is placed in /overlay/robase/root/ (managementinterface.go handleUpdatePostRequest())
1. Stratux reboots
1. stratux-pre-start.sh runs at boot
1. If the <font style='background: purple'>DEB</font> / <font style='background: green'>US</font> is found, it is moved from /boot/firmware/StratuxUpdates/ to  /root/
1. A <font style='background: purple'>DEB</font> is installed via 'dpkg -i'
1. A <font style='background: green'>US</font> is executed
1. The <font style='background: purple'>DEB</font> / <font style='background: green'>US</font> is deleted
1. Stratux reboots again
1. Updated Stratux software starts
