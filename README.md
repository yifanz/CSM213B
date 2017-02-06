# CS213B/EE202B
UCLA CS213B/EE202B Winter 2017 - Prof. Mani Srivastava

Yan Zhang

Yi-Fan Zhang (yifanz@ucla.edu)

#### Table of Contents

1. [Project Proposal](#project-proposal)
2. [Getting Started](#getting-started)
3. [References](#references)
4. [Attributions](#attributions)

## Project Proposal

## Getting Started

### OS and toolchain

Ubuntu 14.04 Desktop Edition.

```
$ sudo apt-get install dfu-util
$ sudo apt-get install gcc-arm-none-eabi

$ dfu-util -V
dfu-util 0.5

(C) 2005-2008 by Weston Schmidt, Harald Welte and OpenMoko Inc.
(C) 2010-2011 Tormod Volden (DfuSe support)
This program is Free Software and has ABSOLUTELY NO WARRANTY

dfu-util does currently only support DFU version 1.0

$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (4.8.2-14ubuntu1+6) 4.8.2
Copyright (C) 2013 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

```

You should already have python on your system. However many of the python scripts depend on the `numpy` library.

```
$ python --version
Python 2.7.6

$ sudo apt-get install python-pip
$ sudo pip install numpy
```

### Getting the source code

This is a private repository and you will need to get added as a collaborator.

```
$ git clone https://github.com/nesl/MobileRadar.git
```

### Flashing the device

There are two methods of loading your program onto the device. You can flash the device directly using the `DFU` over USB or do it over the network with the help of the in-application programmer (IAP). If you are starting fresh, then you should flash the IAP firmware using `DFU` first and then load your custom firmware over the network.

#### Flashing the IAP

You should flash the IAP firmware onto the device first. That way, you can subseqently load you own programs onto the device over the network.

```
$ cd MobileRadar/src/arm/ntb_iap
$ make
```

Connect the board to your computer using the micro-USB located under the UWB radio.
While holding down the DFU button on the board, plug your ethernet cable into the socket near the "PoE Status" LED.
Once the board has power on, the green "PoE Status" LED will light up and you can release the DFU button. Now run this command to verify that your computer has detected the device:

```
$ dfu-util -l
dfu-util 0.5

(C) 2005-2008 by Weston Schmidt, Harald Welte and OpenMoko Inc.
(C) 2010-2011 Tormod Volden (DfuSe support)
This program is Free Software and has ABSOLUTELY NO WARRANTY

dfu-util does currently only support DFU version 1.0

Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=0, name="UNDEFINED"
Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=1, name="UNDEFINED"
Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=2, name="UNDEFINED"
Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=3, name="UNDEFINED"
```

Now you can flash the device.

```
$ sudo make dfu
```

Once the transfer is done, press the reset button.

You can find the IP address of the device by logging into your router (see your router manual).
Optionally, you can assign a static IP to MAC address mapping.
The MAC address of the device follows this convention: `AE:70:00:00:00:<UID>`.
You can configure the `UID` using the small switches on the board under the UWB radio labeled `NODE UID`.
Each switch represents 1 bit ordered from least to most significant.
By convention, the devices are named using the NATO phonetic alphabet.
The `UID` is set to the nth (starting from zero) letter of the alphabet of the device's name.
For example, a device named `India` will have `AE:70:00:00:00:08` and one named `Golf` should be `AE:70:00:00:00:06`.

As a simple test, try connecting to the device over TCP using the `netcat` utility: `nc <IP address of your device> 23458`. Then sending the command `LtR` will toggle the red LED.

#### Loading your firmware

The projects that we will work are located under `MobileRadar/src/arm/ntb_v2` and `MobileRadar/src/arm/watch_v2`.
Assuming that you are running the IAP firmware, you can build the projects locally and load them onto the device over the network.

```
$ cd MobileRadar/src/arm/ntb_v2
$ make
$ ../../client/ntb_firmware_update ./build/ntb_anchor.bin 192.168.50.165
===== NTB Firmware Update: ./build/ntb_anchor.bin --> 192.168.50.165=====
contacting node...
node response received...
rebooting node...
re-establishing connection...
node response received...
locking iap mode...
uploading binary...
booting into application sector...
```

The `ntb_firmware_update` is a python script that will communicate with the IAP and update your firmware over the network.
The script should be invoked with the following pattern `ntb_firmware_update <firmware binary> <IP address of device>`.
Remember to replace the `192.168.50.165` from the example above with the IP address of your device and the `./build/ntb_anchor.bin` with the path of the firmware you want to load.

Upon reset, the device will wait 20 seconds for firmware update requests before entering the currently loaded program.

## References

## Attributions
