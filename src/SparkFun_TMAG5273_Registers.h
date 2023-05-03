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

#ifndef __SparkFun_TMAG5273_Registers_H__
#define __SparkFun_TMAG5273_Registers_H__

enum TMAG5273_Register
{
    TMAG5273_DEVICE_CONFIG_1 = 0X00,
    TMAG5273_DEVICE_CONFIG_2 = 0X01, 
    TMAG5273_SENSOR_CONFIG_1 = 0X02,
    TMAG5273_SENSOR_CONFIG_2 = 0X03,
    TMAG5273_X_THR_CONFIG = 0X04,
    TMAG5273_Y_THR_CONFIG = 0X05,
    TMAG5273_Z_THR_CONFIG = 0X06,
    TMAG5273_T_CONFIG = 0X07,
    TMAG5273_INT_CONFIG_1 = 0X08,
    TMAG5273_MAG_GAIN_CONFIG = 0X09,
    TMAG5273_MAG_OFFSET_CONFIG_1 = 0X0A,
    TMAG5273_MAG_OFFSET_CONFIG_2 = 0X0B,
    TMAG5273_I2C_ADDRESS = 0X0C,
    TMAG5273_DEVICE_ID = 0X0D,
    TMAG5273_MANUFACTURER_ID_LSB = 0X0E,
    TMAG5273_MANUFACTURER_ID_MSB = 0X0F,
    TMAG5273_T_MSB_RESULT = 0X10,
    TMAG5273_T_LSB_RESULT = 0X11,
    TMAG5273_X_MSB_RESULT = 0X12,
    TMAG5273_X_LSB_RESULT = 0X13
    TMAG5273_Y_MSB_RESULT = 0X14,
    TMAG5273_Y_LSB_RESULT = 0X15,
    TMAG5273_Z_MSB_RESULT = 0X16,
    TMAG5273_Z_LSB_RESULT = 0X17,
    TMAG5273_CONV_STATUS = 0X18,
    TMAG5273_ANGLE_RESULT_MSB = 0X19,
    TMAG5273_ANGLE_RESULT_LSB = 0X1A,
    TMAG5273_MAGNITUDE_RESULT = 0X1B
};

#endif
