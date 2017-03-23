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

If you are looking for the __hardware design__ files, they are located here:
```
https://github.com/nesl/MobileRadar/tree/EE202B_winter17/hw_design
```

## Results

> “If I had an hour to solve a problem I'd spend 55 minutes thinking about the problem and 5 minutes thinking about solutions.”    - Albert Einstein

Einstein is reported to have said that if he only had one hour to solve a problem he would spend most of that time analyzing the problem and use the remaining time for solving it.
This quote emphasizes the importance of laying the proper groundwork before rushing into any specific solutions to a problem. 
Although we fell short of acheiving a complete software and hardware implementation of PLoTS, the results from our investigation and the incremental improvements we made to the NTB testbed are still valuable.

With this in mind, we think it is better to view this effort as a precursor for PLoTS.
The project can be roughly divided into two development tracks: hardware/firmware and algorithms/software. In the following sections, we break this down even further to discuss of results on incorporating the custom oscillator, PCB power measurement redesign, power management firmware and the Distributed Kalman Filter implementation.

### Custom Crystal Oscillator

For the board design that incorporates the new oscillator, much of Hani's setup is preserved as per Hani's strong request. However, only the 10MHz is used as the output. Board design aside, since the oscillation frequency and as well as the mechanism to turn off oscillator is changed, modification at the firmware level is needed for configuring the system clock and doing duty cycling with the new crystal. For example, one GPIO is needed to control the ON/OFF of the crystal whereas such function is supported internally through hardware for the internal oscillator. The board size is more than doubled because of all the components that are needed to make the prototype chip working well and to make debug possible when the prototype is not working.

### PCB Modifications

For measuring the power of MCU, a modular, isolated approach is used. A jumper pin is inserted between the MCU supply domain and the main 3.3V domain. The current sensing can be done using any current sensing break-out boards such as the INA219B and the streaming of power data can be collected through devices such as the BBB. The philosophy behind this approach is that no additional board power or CPU cycle is needed for power measurement. The existing power measuring streaming approach seems to burden the CPU so much that processor hang happens occasionally when the stream frequency is high. 

### Power Management Firmware

In terms of power management, we wanted to modify the ntb_v2 firmware such that the microcontroller (MCU) and the DW1000 ultra-wideband (UWB) radio could be duty cycled.
While we were able to successfully put both the MCU and UWB radio to sleep, we ran into difficulty with the wake-up.

#### MCU Sleep

The MCU on the ntb_v2 development board is a STM32F407 ARM Cortex-M4. According to the manual, this class of MCUs from STM features three low-power operating modes<sup>[5](#ref5)</sup>:

* Sleep mode (Cortex®-M4 with FPU core stopped, peripherals kept running)
* Stop mode (all clocks are stopped)
* Standby mode (1.2 V domain powered off)

The Sleep mode can easily be entered using the `__WFI()` macro (wait for interrupt) provided by the hardware abstraction layer (HAL).
This simply halts the MCU until the next interrupt.
The Stop mode powers down the HSE (high-speed external clock) in addition to halting the processor.
In this mode, interrupts will not wake the MCU. Instead, we configured an EXTI line via the NVIC to wake the processor.
Before entering Stop mode, we set the RTC (realtime clock) to assert the EXTI line after some number of cycles.
While the MCU can sleep and wake without problems, the off-chip peripherals (ethernet switch and UWB radio) enter a corrupted state upon wake-up.
We suspect two main causes. First, missing interrupts from the peripheral while the MCU is asleep could cause the peripheral to enter an error state. Second, there may be some misconfiguration of on-chip serial (SPI, I2C) controllers upon wake-up that leads to subsequent miscommunication with off-chip peripherals.
We didn't consider the Standby mode because this will cause SRAM contents to become lost and the MCU will essentially soft-reset on wake-up.
Persisting the memory state to flash before sleeping and then writing custom reset handlers to re-initialize the state is out of scope for this project.

#### UWB Radio Sleep

The UWB radio has a rich set of operating modes (SLEEP, IDLE, SNOOZE, RX, etc) along with several recommended duty cycling regimes<sup>[6](#ref6)</sup>.
This includes a low-power listening mode where the radio is predominantly in the SLEEP state and only wakes periodically to sample the media for preamble sequences.
In this mode, the radio uses a two-phase listen to minimize missing too many messages from a sender.
Between each long SLEEP period, the radio will alternate between IDLE-RX-SNOOZE-IDLE-RX to listen for a preamble.

Similar to the case with the MCU, we were able to configure the radio for sleep, but we were not able to properly restore the radio's state upon wake-up. We feel this is most likely a result of misconfiguration.
The primary challenge with working with the radio is understanding the low level HAL as well as learning how to configure the hardware registers. Looking back, building a good library or abstraction layer for the radio alone would have been a sufficiently challenging project.

### Distributed Kalman Filter Implementation

The Distributed Kalman Filter algorithm (DKF) is a pre-requisite for PLoTS.
This algorithm is responsible for doing the localization, time synchronization and producing the covariance metric which is needed for guiding the duty cycling times.
However, DKF is only implemented in MatLab. Our goal was to port the MatLab DKF implementation to C++ such that it could be executed on a MCU.
This is a duanting task for many reasons.
Firstly, our MCU is limited to ~200 KB program and data memory and the MatLab code is not written with memory optimization in mind.
Second, the algorithm makes use of linear math operations on not only floating point numbers, but also complex numbers.
Third, the development cycle (compile, build, test) on the ntb board is on the order of 30 seconds to a minute and the debugging and logging facilities on the board are lacking.
To overcome these problems, we decided to split the DKF implementation into two sub-projects: libdkf and dkf_sim.

#### libdkf

Libdkf is written as a modern cross-platform C++ library which implements the core DKF algorithm.
We leverage the C++ standard library for its implementation of complex numbers and common data structures.
We also use a modern C++11 compliant compiler to enable better language level mapping between MatLab and C++ (e.g. lambda functions).
Having C++ also enables access to rich open source libraries. Here, we use the Eigen linear math library because of its maturity and portability across different architectures.

In order to be compatible with the existing firmware on the ntb board written in C, we expose an "extern C" API.
This way, we can use the C++ features from within libdkf without requiring the consumer of libdkf to also write in C++.
To do this, we added a separate build step for the ntb firmware which compiles libdkf as a static library and then links it with the existing ntb firmware.
One problem we encountered was the need to POSIX/libc style syscall symbols to be present in order to compile the C++ standard library and Eigen. To get around this, we had to implement stub syscall functions and link them with the firmware. Most of these functions are no-ops and this seems to be fine in our initial tests.

#### dkf_sim

Dkf_sim allows us to test libdkf on the PC using offline measurement data similar to the MatLab implementation.
This allows us to develop the code while leveraging the rich set of debugging tools on the PC.
We also developed a 3D visualization tool to help break our dependence on MatLab's plotting packages.
Currently, dkf_sim is able to process the full set of offline measurement data, but the numeric results drift further away from MatLab's as the calculation progresses.
We suspect two reasons for the drift: high-level logic bugs in the implementation and the fact that we are not accounting for the bias correction logic that the MatLab code applies.
Fixing these issues will be addressed in future work.

## Future Work

PLoTS is a combination of three projects with each one having the scope and complexity of an entire term project. In retrospect, we grossly underestimated this and it would have probably been more productive to put our focus on a single one rather than attempt all three simultaneously.

* DKF Algorithm Implementation
* Hardware Design (Oscillator + Power Measurement)
* Power Management Firmware

In terms of the DKF implementation, we discovered too late that there exists a MatLab Coder plugin which can be used to translate MatLab script to C and C++.
Looking back, this would have been a better approach than what was actually did; translates roughly 535 lines of MatLab to 1062 lines of C++.
Using the codegen feature of MatLab Coder, one could automatically convert MatLab scripts into C/C++ at a per function granularity.
However, this is not a silver bullet solution.
The codegen requires explicit types for arguments to functions that it is not able to automatically deduce and the existing DKF Matlab codebase makes heavy usage of dynamic types making it especially ill-suited for codegen.
Despite this, we still feel that codegen is the way forward. Since one-shot code conversion is clearly not feasible, we recommend incrementally converting the existing MatLab code to be stricly compliant with the codegen.
Using the MEX feature of MatLab (allows MatLab code to call into C/C++), we replace each function in the algorithm one at a time. For each replacement, we test inside MatLab to ensure numeric accuracy.

From a chip design point of view, if any new custom design is intended to be used robustly in a system, then the following must be taken into consideration:

* Simple and robust power supply strategy 
* Self-sustained biasing
* Output swing that is compatible with the MCU

Taking care of the above three will simplify the board design substantially and potentially keep the original board size. 

In terms of energy optimization and power management, we think that the conventional low power techniques have not been fully deployed on the existing NTB hardware/software platform. From a different perspective, conventional techniques saves CPU by outsourcing peripherals whereas for NTB, care must be given to the peripheral, which are mainly UWB radio and ethernet switches, so that they are in a good state before and after sleep. In some sense, dealing with the ntb platform may not be worth the initial productivity cost. It may have been more effective to do the initial development on a commerical well-supported platform (mbed, beaglebone, etc) so our resources can be free to focus more on the UWB radio power management.

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
5. RM0090 Reference manual for STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced ARM-based 32-bit MCUs. <a name="ref5"></a>
6. DW1000 USER MANUAL<a name="ref6"></a>

## Attributions

This project uses open source code. The source code and
corresponding license information is listed below.

1. The DKF implementation uses the C++ Eigen linear math library [Eigen](http://eigen.tuxfamily.org). [[MPL2 License]](http://eigen.tuxfamily.org/index.php?title=Main_Page#License) [[source code]](https://bitbucket.org/eigen/eigen/src)
2. The DKF simulator [Three.js](https://threejs.org) for the 3D visualizations. [[MIT License]](https://github.com/mrdoob/three.js/blob/dev/LICENSE) [[source code]](https://github.com/mrdoob/three.js)

We like to thank Manoj Nagendiran for coming in on weekends to help us setup the project, Paul Martin for answering our questions about the NTB board design as well as the firmware used for this project, Hani Esmaeelzadeh for his work on the pre-charing crystal oscillator and lastly, Amr for being our mentor on this project.
