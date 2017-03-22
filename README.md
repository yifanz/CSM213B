# CS213B/EE202B
UCLA CS213B/EE202B Winter 2017 - Prof. Mani Srivastava

Yan Zhang (charlesz1112@gmail.com)

Yi-Fan Zhang (yifanz@ucla.edu)

#### Table of Contents

1. [Project Proposal](#project-proposal)
    * [Objectives](#high-level-objectives)
2. [Project Files](#project-files)
3. [Results](#results)
    * [Custom Crystal Oscillator](#custom-crystal-oscillator)
    * [PCB Modifications](#pcb-modifications)
    * [Firmware](#power-management-firmware)
    * [DKF Implementation](#distributed-kalman-filter-implementation)
4. [Future Work](#future-work)
5. [Weekly Updates](#weekly-updates)
6. [Developer Setup](#getting-started)
7. [References](#references)
8. [Attributions](#attributions)

## Project Proposal

### PLoTS: Power Efficient Localization and Time Synchronization<sup>[1](#ref1)</sup>

Maintaining accurate time synchonization and localization is a common requirement for many networked embedded systems.
Often the only available means of lowering error is by increasing communication between devices at the expense of energy efficiency.
This is highly undesirable for energy-constrained systems as well as large scale deployments where the energy cost becomes magnified.

While some messaging between devices is inevitable in the system we are investigating, we believe that by judiciously duty cycling the operation of the system, we can mitigate the energy overhead without significantly degrading the accuracy of the localization and time synchronization.
Recent works have proposed various forms of duty cycling to reduce energy consumption in similar types of networks<sup>[2](#ref2),[3](#ref3)</sup>.
The consensus is to turn off the radio during idle times and reactivate it exactly before the next transmission or reception.
This approach depends heavily on the system's accuracy of time synchronization.
However, in practice, it is often not feasible to have near perfect (sub-microsecond) synchronization and we must take into account the error.

Our aim is to enhance our indoor localization system to be able to adapt its operation based on the current synchronization error.
Preliminary offline simulations strongly suggest that the Kalman filter covariance is a good indicator for the state of synchronization of the system<sup>[1](#ref1)</sup>.
By exposing this information to the control system in realtime, we could adjust the messaging rate with reference to a desired confidence threshold and predict the optimal sleep and wake schedule for each device.

We propose to do a hardware and software implemention of the PLoTS<sup>[1](#ref1)</sup> based on the ntb_v2 development board and crazyflie quadracopter.
Both systems are equipt with a DecaWave ultra-wideband radio which will be used perform time synchronization and localization.
We will develop the firmware for adaptively duty cycling the radio and microcontroller according to the covariance calculated from the Kalman filter algorithm.
We can holistically measure the energy consumption of the quadracopter based on flight time; however, PCB level modifications may be needed to instrument the ntb_v2 anchor board.
As an additional optimization, we are considering replacing the conventional onboard oscillator with a novel pre-energizing crystal resonator to minimize the wake up latency of the system to help achieve the maximum energy efficiency.
At the end, we will compare the energy consumption and synchronization error of adaptive duty cycling using PLoTS over a range of error tolerance thresholds against various fixed duty cycling frequencies.

### High-Level Objectives

* Write the power management firmware for entering low power states microcontroller and duty cycling the radio.
* Add duty cycling awareness to the synchronization protocol.
* Expose the Kalman filter covariance to the firmware and adapt the messaging rate accordingly.
* Instrument the testbed to measure power consumption.
* Integrate the pre-energizing crystal resonator.
* Setup and run a complete testbed environment to benchmark the energy efficiency and synchronization error.

## Project Files

Presentation slides [[pdf](https://github.com/yifanz/CSM213B/blob/master/slides.pdf), [keynote](https://github.com/yifanz/CSM213B/blob/master/slides.key)].

Source code for the project is tracked under two __private__ repositories:

__Distributed Kalman Filter C++ Implementation__
```
https://github.com/yifanz/nesl-dkf
```
__NTB Firmware__ (`EE202B_winter17` branch of the NESL MobileRadar project)
```
https://github.com/nesl/MobileRadar/tree/EE202B_winter17
```

## Results

> “If I had an hour to solve a problem I'd spend 55 minutes thinking about the problem and 5 minutes thinking about solutions.”    - Albert Einstein

Einstein is reported to have said that if he only had one hour to solve a problem he would spend most of that time analyzing the problem and use the remaining time for solving it.
This quote emphasizes the importance of laying the proper groundwork before rushing into any specific solutions to a problem. 
Although we fell short of acheiving a complete software and hardware implementation of PLoTS, the results from our investigation and the incremental improvements we made to the NTB testbed are still valuable.

With this in mind, we think it is better to view this effort as a precursor for PLoTS.
We divided this project into two development tracks: hardware/firmware and algorithms/software.

### Custom Crystal Oscillator

TODO

### PCB Modifications

TODO

### Power Management Firmware

### Distributed Kalman Filter Implementation

## Future Work

TODO

## Weekly Updates

### Week 5 (Feb 6 - Feb 12)

* Write proposal.
* Setup development environment and repository branch.
* Setup test environment in the lab (thanks to Amr and Manoj for coming in on the weekend).
* Proposed hardware changes to measure hardware power consumption.
* Requested custom crystal from Dr. Pamarti's group.

### Week 6 (Feb 13 - Feb 19)

* Setup the testbed for localization and time synchronization.
* Implement entering and exiting from lower power MCU states.

### Week 7 (Feb 20 - Feb 26)

* Implement a communication protocol between nodes for collecting and broadcasting stats.

### Week 8 (Feb 27 - Mar 5)

* Understand the Matlab code for the Distributed Kalman Filter algorithm.
* Add C++ toolchain and build scripts to prepare for porting DKF to the ntb board.

### Week 9 (Mar 6 - Mar 12)

* Port DKF from Matlab to C++.
* Implement UWB radio low power modes.

### Week 10 (Mar 13 - Mar 19)

* Continue debugging DKF algorithm.
* Continue debugging low power modes.
* Prepare presentation and write report.

## Getting Started

### OS and toolchain

Ubuntu 14.04 Desktop Edition.

```
$ sudo apt-get install dfu-util
$ sudo apt-get install gcc-arm-none-eabi
$ sudo apt-get install curl

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
$ sudo pip install pyserial
```

### Getting the source code

MobileRadar is a private repository and you will need to get added as a collaborator. All of the code for this project is in the `EE202B_winter17` branch.

```
$ git clone https://github.com/nesl/MobileRadar.git
cd MobileRadar
git checkout EE202B_winter17
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

1. Hani Esmaeelzadeh and Amr Alanwar. "PLoTS: Power Efficient Localization and Time Synchronization" <a name="ref1"></a>
2. Zhen, ChengFang, et al. "Energy-efficient sleep/wake scheduling for acoustic localization wireless sensor network node." International Journal of Distributed Sensor Networks 2014 (2014). <a name="ref2"></a>
3. Wu, Yan, Sonia Fahmy, and Ness B. Shroff. "Optimal QoS- aware sleep/wake scheduling for time-synchronized sensor networks." 2006 40th Annual Conference on Information Sciences and Systems. IEEE, 2006. <a name="ref3"></a>
4. Federico S. Cattivelli and Ali H. Sayed. "Distributed Nonlinear Kalman Filtering With Applications to Wireless Localization." Department of Electrical Engineering University of California, Los Angeles. <a name="ref4"></a>

## Attributions

This project uses open source code. The source code and
corresponding license information is listed below.

1. The DKF implementation uses the C++ Eigen linear math library [Eigen](http://eigen.tuxfamily.org). [[MPL2 License]](http://eigen.tuxfamily.org/index.php?title=Main_Page#License) [[source code]](https://bitbucket.org/eigen/eigen/src)
2. The DKF simulator [Three.js](https://threejs.org) for the 3D visualizations. [[MIT License]](https://github.com/mrdoob/three.js/blob/dev/LICENSE) [[source code]](https://github.com/mrdoob/three.js)

We like to thank Manoj Nagendiran for coming in on weekends to help us setup the project, Paul Martin for answering our questions about the NTB board design as well as the firmware used for this project, Hani Esmaeelzadeh for his work on the pre-charing crystal oscillator and lastly, Amr for being our mentor on this project.
