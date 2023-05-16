# Open-radar

## Introduction

This is a repository for open source radar project. There is an implementation of a simple FMCW radar for Arduino. It uses triangular wave modulation pattern. However the functionality of the system is very limited due to the ATmega328p. To work properly, this would require a proper hardware, specified for digital signal processing, like FPGA or DSP. The main purpose of this project is to provide a simple and easy to understand implementation of a radar system.

## Hardware

The hardware used for this project is Arduino Nano and AgilSense AP88 radar.

## Documentation

`main.c` - main file of the project.
`main.h` - header file for main.c.

`void initRadar()` - initializes the radar. Sets EN pin to high and sends SPI commands to the radar.

`void initDAC()` - initializes the DAC. Sets DAC voltage to 0V.

`void meassureFrequency()` - measures the frequency of the received signal. It uses `pulseIn()` function to measure the time of rising and falling edges of the signal. Then it calculates the frequency.

`void displayData()` - displays the meassured frequency in Hz and speed in km/h on the serial monitor.

`void displayAdvancedData()` - displays speed in km/h and distance in meters on the serial monitor.

`void fmSweep()` - performs a single sweep of the radar along with frequency meassurement

`void calculateSpeed()` - calculates the speed of the object based on the frequency shift.

`void calculateDistance()` - calculates the distance of the object based on the frequency shift.

`void enablePa()` - enables the power amplifier of the AP88.

`void disablePa()` - disables the power amplifier of the AP88.
