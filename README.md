# CS213B
UCLA CS213B/EE202B Winter 2017

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

### Getting the source code

```
$ git clone https://github.com/nesl/MobileRadar.git
$ cd MobileRadar/src/arm/ntb_iap
$ make
```

### Flashing the device

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

You can try connecting to the device over TCP using the `netcat` utility: `nc <IP address of your device> 23458`. Then sending the command `LtR` will toggle the red LED.
