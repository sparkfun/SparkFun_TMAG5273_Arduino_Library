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

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_TMAG5273_Registers.h"
#include "SparkFun_TMAG5273.h"

TMAG5273::TMAG5273()
{
}

/* BEGIN
    This function checks if the TMAG will ACK over I2C, and 
    if the TMAG will correctly self-identify with the proper 
    device ID. This will set the address of the deivce along 
    with setting the wire for the I2C Communication. 
    This will return true if both checks pass. 
*/
bool TMAG5273::begin(uint8_t sensorAddress, TwoWire &wirePort)
{
	_i2cPort = &wirePort;			// Chooses the wire port of the device
	_deviceAddress = sensorAddress; // Sets the address of the device

	//make sure the TMP will acknowledge over I2C
	_i2cPort->beginTransmission(_deviceAddress);
	if (_i2cPort->endTransmission() != 0)
	{
		return false;
	}

	uint16_t deviceID = readRegister(TMAG5273_DEVICE_ID); // reads registers into rawData

	//make sure the device ID reported by the TMP is correct
	//should always be 0x0117
	if (deviceID != DEVICE_ID_VALUE)
	{
		return false;
	}

	return true; //returns true if all the checks pass
}


/* GET ADDRESS 
	This function returns the address of the device once
	called upon. This address can only be 0x22.
*/
uint8_t TMP117::getAddress()
{
	return _deviceAddress;
}


/* READ TEMPERATURE
  - Enable temperature bit from T_CONFIG Register
  - Read temperature back
*/
float TMAG5273::readTemperature(void)
{
    
}