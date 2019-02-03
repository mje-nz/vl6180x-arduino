# VL6180X library for Arduino

[![Build Status](https://travis-ci.org/mje-nz/vl6180x-arduino.svg?branch=master)](https://travis-ci.org/mje-nz/vl6180x-arduino)<br/>

Fork of the [Pololu VL6180X library](https://github.com/pololu/vl6180x-arduino) which cleans things up a bit and will soon add support for non-blocking reads on Teensy using [i2c_t3](https://github.com/nox771/i2c_t3).

## Summary

This is a library for the Arduino IDE that helps interface with ST's [VL6180X time-of-flight distance and ambient light sensor](https://www.pololu.com/product/2489).  The library makes it simple to configure the sensor and read range and ambient light level data from it via I&sup2;C.

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.6.x or later; we have not tested it with earlier versions.  This library should support any Arduino-compatible board, including the [Pololu A-Star 32U4 controllers](https://www.pololu.com/category/149/a-star-programmable-controllers).

## Getting started

### Hardware

A [VL6180X carrier](https://www.pololu.com/product/2489) can be purchased from Pololu's website.  Before continuing, careful reading of the [product page](https://www.pololu.com/product/2489) as well as the VL6180X datasheet and application notes is recommended.

Make the following connections between the Arduino and the VL6180X board:

#### 5V Arduino boards

(including Arduino Uno, Leonardo, Mega; Pololu A-Star 32U4)

    Arduino   VL6180X board
    -------   -------------
         5V - VIN
        GND - GND
        SDA - SDA
        SCL - SCL

#### 3.3V Arduino boards

(including Arduino Due)

    Arduino   VL6180X board
    -------   -------------
        3V3 - VIN
        GND - GND
        SDA - SDA
        SCL - SCL

### Software

If you are using the [Arduino IDE](http://www.arduino.cc/en/Main/Software):

1. Download the [latest commit from GitHub](https://github.com/mje-nz/vl6180x-arduino/archive/master.zip) and decompress it.
2. Rename the folder "vl6180x-arduino-master" to "VL6180X".
3. Move the "VL6180X" folder into the "libraries" directory inside your Arduino sketchbook directory.  You can view your sketchbook location by opening the "File" menu and selecting "Preferences" in the Arduino IDE.  If there is not already a "libraries" folder in that location, you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

If you are using Platformio, just add `VL6180X=https://github.com/mje-nz/vl6180x-arduino.git` to your lib_deps.

To use i2c_t3 instead of Wire, add `-DVL6180X_USE_I2C_T3` to your build or edit the top of `VL6180X.h`.

## Examples

Several example sketches are available that show how to use the library. You can access them from the Arduino IDE by opening the "File" menu, selecting "Examples", and then selecting "VL6180X". If you cannot find these examples, the library was probably installed incorrectly and you should retry the installation instructions above.

## Version history

My versions:

* 0.1 (2018 Feb 3): Big cleanup, use begin() instead of init() and configureDefault(), preliminary support for i2c_t3.

Pololu versions:

* 1.2.0 (2016 May 18): Added functions for reading range in millimeters, taking range scaling factor into account. Changed example sketches to use these functions.
* 1.1.0 (2016 May 12): Added functions to set range scaling factor and example sketch to demonstrate scaling.
* 1.0.1 (2016 Mar 14): Added missing `Serial.begin()` to examples and changed `configureDefault()` to reset some additional registers to power-on defaults.
* 1.0.0 (2015 Sep 24): Original release.
