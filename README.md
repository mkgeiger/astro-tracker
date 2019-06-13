# High precision Astro Tracker with wireless GOTO

## Overview

The inspiration of this project founded in the combination of my two hobbies: astronomy and microcontroller projects. This projects describes building an equatorial mount stepper motor controller with GPS high precision star-, moon- and sun-tracking. A wireless GOTO control via Skysafari App and a webinterface for individual stepper motor settings is also implemented. The features for my needs are:
* GPS timebase for high precision tracking
* tracking of rectascension in stellar, lunar, solar and user defined speeds
* north and south hemisphere support
* wireless GOTO over WIFI supporting LX200 protocol from the Skysafari App
* stepper motor control for rectascension and declination axis including fast slewing for GOTO
* LED indication of fast slewing in action
* individual stepper motor settings over a webpage presented by a webserver
* autoguider port
* 12V power supply

## Hardware

### NodeMCU ESP32

I decided to take this microcontroller board because of its high power with 2 cores, 4 fast high precision couters and easy programming, WIFI and small form factor. Following GPIOs are used:

  Signal          | ESP32
  ----------------|-----------
  RA_PHASE1..4    | GPIO12..15
  DEC_PHASE1..4   | GPIO16..19
  RA_LED          | GPIO0
  DEC_LED         | GPIO4
  STATUS_LED      | GPIO2
  ONE_PPS         | GPIO21
  AUTOGUIDE_RA_M  | GPIO26
  AUTOGUIDE_RA_P  | GPIO27
  AUTOGUIDE_DEC_M | GPIO32
  AUTOGUIDE_DEC_P | GPIO33
  
This module can be obtained at Ebay for about 5€. ![ESP32 NodeMCU](/datasheets/ESP32S_pinout.jpg)

### GPS receiver Navilock EM406A

For calibrating the CPU frequency to less than 1ppm a high precision timebase is needed. The most accurate timebase which a normal person can get is from GPS. Some of these embedded GPS devices feed out a 1PPS (pulse per second) pin. This pin delivers a high accurate pulse per second with extremely low jitter. For this project only this pin is used for CPU counters calibration. The UART pins are connected and can be used for upcoming features. The datasheet for the EM406A can be found [here](/datasheets/EM406A_User_Manual.pdf).

![gps_EM406A](/hardware/gps_EM406A.jpg)

### Stepper motor controller

The TB6612FNG is a great motor driver that is perfect for interfacing a single bipolar/unipolar stepper motor. The MOSFET-based H-bridges are much more efficient than the BJT-based H-bridges used in older drivers such as the L298N, which allows more current to be delivered to the motors and less to be drawn from the logic supply. This little breakout board gives direct access to all of the features of the TB6612FNG and adds power supply capacitors and reverse battery protection on the motor supply.

![pololu_TB6612FNG](/hardware/pololu_TB6612FNG.jpg)

![pololu_schematic](/datasheets/pololu_schematic.png)

### Vixen MT-1 stepper motor

In my mount (Vixen GP) two of these motors are used for rectascension and declination. This stepper motor has following data:
* motor max speed factor: 32
* motor steps per full turn: 48
* gear box ratio: 1:120
* worm wheel ratio (Vixen GP mount): 1:144

![vixen_mt1](/hardware/vixen_mt1.jpg)

![mt1_stepper_connector](/datasheets/mt1_stepper_connector.png)

### Power supply unit

This DC/DC converter is a cheap step down module based on the LM2596 chip. This module can be obtained at Ebay for about 1€. The input voltage depends on the stepper motors, for the Vixen MT-1 motors it is 12V. The output voltage has to be trimmed to 5V.

![power_LM2596S](/hardware/power_LM2596S.jpg)

### Connectors

#### Stepper motors (SUB-D 15)

![connectors](/datasheets/connectors.png)

#### Autoguider port ST-4 (RJ-11/12)

![st4_autoguiding](/hardware/st4_autoguiding.png)

### PCB layout

The PCB layout is created with Eagle Autorouter in a way to place all modules mentioned above near to each other on a single PCB. The Eagle PCB file can be downloaded from [here](/hardware/astrotracker.brd).

![pcb](/hardware/pcb.png)

## [Software](/AstroTracker/AstroTracker.ino)

### Initialization

#### Shared memory between tasks

For this purpose a mutex semaphore is created.

#### Exchanging data between tasks on different cores

For sending LX200 messages to the process task a message queue is created.

#### GPIOs

Most pins are configured as output pins, except the autoguiding and the 1PPS pin. These are configured respectively as input pins with pullups and as interrupt pin.

#### Non volatile memory

All configured motor-, gear- and speed-parameters are read during every startup.

#### High precision HW counters frequency measurement

At every startup the clock frequency of the four 64-bit HW counters is measured. This is done with help of the 1PPS pulse from the GPS module. Every precise second (1ppm jitter) the counter values are read out to determine the counting frequency. 10 values are taken to calculate the average counting frequency. This value is taken to calculate the exact parameterisation of the different stepping speeds.

#### Timers

For reaching the highest precision in tracking all four 64-bit timers are used. 
For further information see chapter [Interrupts].

#### Parameterisation

For the parameterisation the values from the non volatile memory (motor-, gear- and speed parameters) and the determined counting clock frequency are used for calculating the different stepping speeds.

#### WIFI

WIFI is initialized as an access point to be able to connect with a mobile phone also without an existing network in the field. The access parameters are: IP: 192.168.4.1, SSID: ASTRO_TRACKER, Password: see source code (default: xxxxxxxx).

### Tasks

The overall architecture splits the SW into two parts running on the two independant CPU cores. I decided to run the server tasks on core 0 (Pro CPU) and the time critical tasks and interrupts on core 1 (App CPU). 

#### LX200 server task (core 0, low priority)

This tasks implements a WIFI server to accept LX200 commands from a Sky Safari client, like a mobile phone with Sky Safari for Android. Sky Safari does not make a persistent connection, so each command query is managed as a single independent client. Only the LX200 commands (like GOTO, ALIGN, STOP, etc.) which are sent by Sky Safari are supported actually. The commands are parsed to extract informations like coordinates which are sent via message queue to the "Process Commands" task running on core 0. The northern or southern hemisphere parameter is determined by the "current site latitude" LX200 command.

#### Web server task (core 0, high priority)

This task offers a webpage to any client. In a webbowser the user can configure the individual parameters of the stepper motors, like steps per rotation and maximum speed factor. Also the gear and worm wheel ratio can be configured. Also the user has the posibility to choose the tracking speed of the rectascenson axis. All these parameters are stored in a non volatile memory to be present with the next boot of the system.

![webpage](/photos/webpage_settings.png)

#### Process commands task(core 1, high priority)

This task is responsible for receiving the parsed LX200 commands and for calculating the stepping control parameters for the stepper motors, like stepping speed, direction up/down and left/right and the number of steps for fast foreward/backward slewing (GOTO). For determining the fast foreward/backward steps in rectascension direction, the time for slewing to the destination object and the rotation of the earth around the rectascension axis are considered in the calculation formulas.

#### Current position task (core 1, low priority)

This task is responsible for sending the actual position (rectascension and declination) back to the Sky Safari client. Sky Safari uses this information to draw a crosshair on the sky where the telescope is pointing to. An update of this information is sent every 100ms.

#### Autoguider task (core 1, loop(), very low priority)

This task checks continuously the autoguider ports to correct the deviation with 0.5x speed in DEC-, DEC+, RA- direction and 1.5x speed in RA+ direction. Apart from this this task is also responsible to let the alive LED blink with 1Hz.

### Interrupts

#### 1 PPS interrupts

This interrupt is triggered every second by the GPS module. It used initially for calibrating the high resolution counters. At startup 10 samples of counter values are taken every second to calculate the average counting frequency. This is needed to reach a high accuracy.

#### Normal speed timer interrupt

This timer is programmed to come for every single full step for the rectascension stepper motor when tracking is active. Tracking speeds can be stellar, lunar, solar and various user defined speeds.

#### Fast speed timer interrupt

This timer is programmed to come for every single full step for both rectascension and declination stepper motors when the user presses the up/down/left/right buttons or when slewing to a destination object is currently active (GOTO). The slewing speed is the maximum possible speed for the motors, in case of MT-1 motors it is 32x.

#### Correction RA+ interrupt

This timer is programmed to come for positive rectascension autoguiding corrections. The speed is then with factor 1.5x.

#### Correction RA-, DEC-, DEC+ interrupt

This timer is programmed to come for negative rectascension, negativ- and positive declination autoguiding corrections. The speed is then with factor 0.5x.

## Photos

Assembled PCB:
![pcb](/photos/pcb.jpg)

Opened case:
![open1](/photos/open1.jpg)

![open2](/photos/open2.jpg)

Finished device:
![closed](/photos/closed.jpg)

System view (incl. Vixen GP mount):
![system](/photos/system.jpg)
