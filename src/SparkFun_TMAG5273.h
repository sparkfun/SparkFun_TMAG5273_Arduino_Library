/******************************************************************************
SparkFunTMAG5273.cpp
SparkFunTMAG5273 Library Source File
Madison Chodikov @ SparkFun Electronics
Original Creation Date: May 1st, 2023
https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library

This file implements all functions of the TMAG5273. Functions here range
from ________

Development environment specifics:
	IDE: Arduino 1.8.9
	Hardware Platform: Arduino Uno
	TMAG5273 Breakout Version: 1.0.0
This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SparkFun_TMAG5273_H__
#define __SparkFun_TMAG5273_H__

#include <Wire.h>
#include <Arduino.h>
#include "SparkFun_TMAG5273_Registers.h"

#define DEVICE_ID_VALUE 0x22    // Value found in the device ID register


class TMAG5273
{
public: 
    TMAG5273(); // Constructor

    bool begin(uint8_t sensorAddress = 0x48, TwoWire &wirePort = Wire); // Checks for ACK over I2C, and sets the device ID of the TMAG and chooses the wire port
    uint8_t getAddress();                                               // Returns the address of the device 

    float readTemperature(void);                                        // Returns temperature of device
    uint8_t readMagField(float* Bx, float* By, float* Bz);              // Returns the magnetic field of the device - precise up to +/-1mT




private: 
    TwoWire *_i2cPort = NULL;
    uint8_t _deviceAddress;
    
    uint16_t readRegister(uint8_t reg);             // Reads 2 register bytes from sensor
    void writeRegister(uint8_t reg, uint16_t data); // Wires single byte of data to the sensor

}

#endif