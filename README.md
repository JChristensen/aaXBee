# Double-A XBee Sensor Node Sketch
https://github.com/JChristensen/aaXBee  
README file  
Jack Christensen  
Apr 2019  

## License
Arduino Double-A XBee Sensor Node Sketch Copyright (C) 2019 Jack Christensen GNU GPL v3.0

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License v3.0 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/gpl.html>

## Introduction
Firmware for the [Double-A XBee Sensor Node](https://github.com/JChristensen/aaXBee_HW).

Program this sketch with ICSP, no bootloader is used.

Add the following to the boards.txt file (hardware/arduino/avr/boards.txt):

```
##############################################################

aaDL.name=Double-A Datalogger/Double-A XBee Node

aaDL.upload.tool=avrdude
aaDL.upload.protocol=arduino
aaDL.upload.maximum_size=32768
aaDL.upload.maximum_data_size=2048

aaDL.bootloader.low_fuses=0x7f
aaDL.bootloader.high_fuses=0xde
aaDL.bootloader.extended_fuses=0x06

aaDL.build.mcu=atmega328p
aaDL.build.f_cpu=8000000L
aaDL.build.board=AVR_AADL
aaDL.build.core=arduino
aaDL.build.variant=standard

##############################################################
```