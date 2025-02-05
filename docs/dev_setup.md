## Choices...
If you want to start working on the Stratux code base, here are a few hints for a developer-friendly setup.
Note that this only works as-is for the Stratux EU Edition.

You basically have two choices.
- Remotely working on a PI
- Working locally on a Linux machine

Working remotely on a pi can also be done from Windows and has the advantage of full hardware support (e.g. AHRS module, which can't be plugged into your PC).
Working locally on a Linux machine is a bit harder to set up and requires some creativity, but is much faster and more comfortable when doing basic work that doesn't rely on AHRS/Baro stuff.

As of Stratux-EU 025, we recommend working remotely on the Pi, preferably a Pi4.

## Remote Setup
To work remotely on the PI, first, **make sure your Stratux is connected to the internet**. You will also have to **Enable persistent logging on the web interface** to make sure that the file system of the PI is writable.

Additionally, keep in mind that the PI's CPU is clocked down by default in the Stratux image to save some power. This makes compiling stuff extremely slow. Therefore it is recommended to remove/comment the *_freq lines in /boot/config.txt (or /boot/firmware/config.txt starting with bookworm) and restarting it.

Then connect to it via Putty/SSH and set a root password (`sudo passwd`).
Now clone the Stratux repository and install some dependencies/go:
```bash
sudo -s
apt install build-essential git
cd /root
git clone --recursive https://github.com/stratux/stratux.git
wget https://golang.org/dl/go1.20.1.linux-arm64.tar.gz
tar xzf go1.20.1.linux-arm64.tar.gz
rm go1.20.1.linux-arm64.tar.gz
```
Afterwards, install Visual Studio Code on your PC, and additionally install and configure the VSCode Remote-SSH Extension: https://code.visualstudio.com/docs/remote/ssh
When done, connect to the RPi as root via Visual Studio Code.
Then click `File->Open Folder` and type in `/root/stratux`.
You should now have the Stratux EU project loaded into VSCode.
There are preconfigured build an debug tasks in the project which allow you to create a debuggable Stratux build and remote-debug it in VSCode.
To get started, you need to install the "Go" extensions ("Install on Stratux" while connected via Remote-SSH).
The extension will ask you to install several tools (including delve if you want to debug). Go ahead and do so.
Before debugging, run `stxstop` to stop the already running stratux service.

There is also a makefile in the repository, so `make && make install && stxrestart` would be a simple way to build, install and start.

## Local x86 Linux Setup
This is an **ADVANCED** setup, which requires experience with the Linux shell, system, and common build processes. Do not open issues if you can't get it to work - you are on your own here.

First, you need to install and setup go and its dependencies.
Once done, you can try to build your first X86 version of Stratux. Clone the Stratux repository and run `make`.
To make everything work, you will also have to run `sudo make install`, so that Stratux can find the OGN decoder, dump1090, etc. This will also install Stratux service files and udev rules. If you don't want that, you can instead run `make optinstall` to only install the stuff that resides in `/opt/stratux`.
Once everything is running, you might want to install Visual Studio Code and open the Stratux directory in there.
There are preconfigured build and debug tasks for VSCode in the repository.
