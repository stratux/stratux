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

1. Update script file is uploaded via the Stratux web interface
1. The file is placed in /overlay/robase/root/
1. Stratux reboots
1. stratux-pre-start.sh runs at boot
1. If the script is found, it is moved from /boot/firmware/StratuxUpdates/ to  /root/
1. The script is executed
1. The script is deleted
1. Stratux reboots again
1. Updated Stratux software starts