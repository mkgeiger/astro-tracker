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

## Software

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
