/******************************************************************************
SparkFunTMAG5273.cpp
SparkFunTMAG5273 Library Source File
Madison Chodikov @ SparkFun Electronics
Original Creation Date: May 1st, 2023
https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library

This file implements all functions of the TMAG5273. Functions here range
from reading the X, Y,and Z Magnetic Fields, to reading and writing various
settings in the sensor.

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

#define DEVICE_ID_VALUE 0x0D    // Value found in the device ID register

// Device Configuration 1 Register - DEVICE_CONFIG_1
typedef union { 
    struct 
    {
        uint8_t CRC_EN : 1;			// Enables I2C CRC byte to be sent
		uint8_t MAG_TEMPCO : 2;     // Temperature coefficient of the magnet
        uint8_t CONV_AV : 3;        // Enables additional sampling of the sensor data to reduce the noise effect
        uint8_t I2C_READ : 2;       // Defines the I2C read mode

    }DEVICE_CONFIG_1_FIELDS;
    uint16_t DEVICE_CONFIG_1_COMBINED;
}DEVICE_CONFIG_1_REG;

// Device Configuration 2 Register - DEVICE_CONFIG_2
typedef union { 
    struct 
    {
        uint8_t THR_HYST : 3;           // Selects thresholds for the interrupt function
        uint8_t LP_LN : 1;              // Selects the modes between low active current or low-noice modes
        uint8_t I2C_GLITCH_FILTER : 1;  // I2C glitch filter
        uint8_t TRIGGER_MODE : 1;       // Selects condition which initiates a single conversion based off already configured registers 
        uint8_t OPERATING_MODE: 2;      // Selects operating Mode and updates value based on operating mode if transition between modes occurs

    }DEVICE_CONFIG_2_FIELDS;
    uint16_t DEVICE_CONFIG_2_COMBINED;
}DEVICE_CONFIG_2_REG;

// Sensor Configuration 1 Register - SENSOR_CONFIG_1
typedef union { 
    struct 
    {
        uint8_t MAG_CH_EN : 4;      // Enables data acquisition of the magnetic axis channels
        uint8_t SLEEPTIME : 4;      // Selects teh time spent in low power mode between conversions
    }SENSOR_CONFIG_1_FIELDS;
    uint16_t SENSOR_CONFIG_1_COMBINED;
}SENSOR_CONFIG_1_REG;

// Sensor Configuration 2 Register - SENSOR_CONFIG_2
typedef union { 
    struct 
    {
        uint8_t RESESRVED : 1;      // Reserved
        uint8_t THRX_COUNT : 1;     // Number of threshold check
        uint8_t MAG_THR_DIR : 1;    // Selects the direction of threshold check 
        uint8_t MAG_GAIN_CH : 1;    // Selects the axis for magnitude gain correction value entered
        uint8_t ANGLE_EN : 2;       // Enables angle calculation, magnetic gain, and offset corrections between 2 selected channels
        uint8_t X_Y_RANGE : 1;      // Select the X, Y axes magnetic range from 2 different options (see table 7-11 in datasheet)
        uint8_t Z_RANGE : 1;        // Selects the Z axis magnetic range from 2 different options (see table 7-11 in datasheet)
    }SENSOR_CONFIG_2_FIELDS;
    uint16_t SENSOR_CONFIG_2_COMBINED;
}SENSOR_CONFIG_1_REG;

// Temperature Configuration Register - T_CONFIG
typedef union { 
    struct 
    {
        uint8_t T_THR_CONFIG : 7;   // Temperature threshold code entered by user
        uint8_t T_CH_EN : 1;        // Enables data acquisition of the temperature channel
    }T_CONFIG_FIELDS;
    uint16_t T_CONFIG_COMBINED;
}T_CONFIG_REG;

// Int Configuration 1 Register - INT_CONFIG_1
typedef union { 
    struct 
    {
        uint8_t RSLT_INT : 1;       // Enable interrupt response on conversion complete
        uint8_t THRSLD_INT : 1;     // Enable interrupt response on a predefined threshold cross
        uint8_t INT_STATE : 1;      // INT interrupt latched or pulsed
        uint8_t INT_MODE : 3;       // Interrupt mode select (see table 7-6 in datasheet for modes)
        uint8_t RESERVED : 1;       // Reserved
        uint8_t MASK_INTB : 1;      // Mask INT pin when INT connected to GND
    }INT_CONFIG_1_FIELDS;
    uint16_t INT_CONFIG_1_COMBINED;
}INT_CONFIG_1_REG;

// I2C Address Register - I2C_ADDRESS
typedef union { 
    struct 
    {
        uint8_t I2C_ADDRESS : 7;            // 7-bit default factory I2C address is uploaded from OTP during first power up
        uint8_t I2C_ADDRESS_UPDATE_EN : 1;  // Enables a new user defined I2C address
    }I2C_ADDRESS_FIELDS;
    uint16_t I2C_FIELDS_COMBINED;
}I2C_FIELDS_REG;

// Converserion Status Register - CONV_STATUS
typedef union { 
    struct 
    {
        uint8_t SET_COUNT : 3;      // Rolling count of conversion data sets
        uint8_t POR : 1;            // Device powered up, or experienced power-on-reset
        uint8_t RESERVED : 2;       // Reserved
        uint8_t DIAG_STATUS : 1;    // Detect any internal diagnostics fail (see table 7-32 in datasheet for more info)
        uint8_t RESULT_STATUS : 1;  // Conversion data buffer is ready to be read
    }CONV_STATUS_FIELDS;
    uint16_t CONV_STATUS_COMBINED;
}CONV_STATUS_REG;

// Device Status Register - DEVICE_STATUS
typedef union { 
    struct 
    {
        uint8_t RESERVED : 3;       // Reserved
        uint8_t INTB_RB : 1;        // Indicates the level that the device is reading back from !INT pin
        uint8_t OSC_ER : 1;         // Indicates if oscillator error is detected
        uint8_t INT_ER : 1;         // Indicates if !INT pin error is detected
        uint8_t OTP_CRC_ER : 1;     // Indicates if OTP CRC error is detected
        uint8_t VCC_UV_ER : 1;      // Indicates if VCC undervoltage was detected
    }DEVICE_STATUS_FIELDS;
    uint16_t DEVICE_STATUS_COMBINED;
}DEVICE_STATUS_REG;


class TMAG5273
{
public: 
    TMAG5273(); // Constructor

    bool begin(uint8_t sensorAddress = 0x48, TwoWire &wirePort = Wire); // Checks for ACK over I2C, and sets the device ID of the TMAG and chooses the wire port
    uint8_t getAddress();                                               // Returns the address of the device 



    float readTemperature(void);                                        // Returns temperature of device
    uint8_t readMagField(float* Bx, float* By, float* Bz);              // Returns the magnetic field of the device - precise up to +/-1mT
    
    // Set Device Configuration Register Settings
    void setCRC(uint8_t crc_mode);                                // Sets the I2C CRC byte to be sent
    void setMagTemp(uint8_t magTemp_mode);                        // Selects the temp coefficient of the magnet
    void setConvAvg(uint8_t avg_mode);                            // Sets additional sampling of the sensor data
    void setIntThreshold(uint8_t threshold);                      // Selects the threshold for the interrupt function (one direction, OR N/S)
    void setLowPower(uint8_t lp_ln_mode);                         // Selects if device is operating in low power or low noise mode
    void setOperatingMode(uint8_t op_mode);                       // Selects operating mode and updates values based on operating mode
    void setMagChannel(uint16_t channel_mode);                    // 
    void setSleeptime(uint16_t sleepTime);                        // 
    void setMagDir(uint8_t thresh_dir);                           // 
    void setMagGain(uint8_t gain_adjust);                         // 
    void setAngle(uint8_t angle_en);                              // 
    void setXYAxisRange(uint8_t XY_axis_range);                   // 
    void setZAxisRange(uint8_t Z_axis_range);                     // 
    void setXThresh(uint8_t X_thresh);                            // 
    void setYThresh(uint8_t Y_thresh);                            // 
    void setZThresh(uint8_t Z_thresh);                            // 
    void setTempThresh(uint8_t T_thresh);                         // 
    void setIntRslt(uint8_t int_en);                              // 
    void setThreshEn(uint8_t en_int_response);                    // 
    void setIntState(uint8_t int_state);                          // 
    void setIntMode(uint8_t config_mode);                         // 
    void setMaskInt(uint8_t int_en);                              // 

    void setI2CAddress(uint16_t address);                         // 



    // Get Device Configuration Register Settings
    uint16_t getCRC();                                            // 
    uint16_t getMagTemp();                                        // 
    uint16_t getConvAvg();                                        // 
    uint16_t getIntThreshold();                                   // 
    uint16_t getLowPower();                                       // 
    uint16_t getOperatingMode();                                  // 
    uint16_t getMagChannel();                                     // 
    uint16_t getSleeptime();                                      // 
    uint16_t getMagDir();                                         // 
    uint16_t getMagGain();                                        // 
    uint16_t getAngle();                                          // 
    uint16_t getXYAxisRange();                                    // 
    uint16_t getZAxisRange();                                     // 
    uint16_t getXThresh();                                        // 
    uint16_t getYThresh();                                        // 
    uint16_t getZThresh();                                        // 
    uint16_t getTempThresh();                                     // 
    uint16_t getTempThreshEN();                                   // 
    uint16_t getIntRslt();                                        // 
    uint16_t getThreshEn();                                       // 
    uint16_t getIntState();                                       // 
    uint16_t getIntMode();                                        // 
    uint16_t getMaskInt();                                        //

    uint16_t getI2CAddress();                                     // 
    uint16_t getDeviceID();                                       // 
    uint16_t getManufacturerID();                                 // 

    uint16_t getTemp();                                           // Uses T_MSB_RESULT and T_LSB_RESULT Registers
    uint16_t getXData();                                          // Uses X_MSB_RESULT and X_LSB_RESULT Registers
    uint16_t getYData();                                          // Uses Y_MSB_RESULT and Y_LSB_RESULT Registers
    uint16_t getZData();                                          // Uses Z_MSB_RESULT and Z_LSB_RESULT Registers


private: 
    TwoWire *_i2cPort = NULL;
    uint8_t _deviceAddress;
    
    uint16_t readRegister(uint8_t reg);             // Reads 2 register bytes from sensor
    void writeRegister(uint8_t reg, uint16_t data); // Wires single byte of data to the sensor

    void configI2C_RD(uint8_t read_mode);

}

#endif