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
    IDE: Arduino 2.1.0
    Hardware Platform: Arduino Uno
    TMAG5273 Breakout Version: 1.0.0
Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SparkFun_TMAG5273_H__
#define __SparkFun_TMAG5273_H__

#include "SparkFun_TMAG5273_Registers.h"
#include <Arduino.h>
#include <Wire.h>

#define DEVICE_ID_VALUE 0x0D // Value found in the device ID register
#define TSENSE_T0 25         // Temperature calculation constant
#define TADC_T0 17508        // Temperature caluclation constant
#define TADC_RES 60.1        // Temperature calculation constant

#define TMAG_OK INT8_C(0) // To define success code

struct TMAG5273_Interface
{
};

class TMAG5273
{
  public:
    TMAG5273(); // Constructor

    int8_t begin(uint8_t sensorAddress = 0x48,
                 TwoWire &wirePort =
                     Wire); // Checks for ACK over I2C, and sets the device ID of the TMAG and chooses the wire port
    uint8_t getAddress();   // Returns the address of the device

    uint8_t readMagField(float *Bx, float *By,
                         float *Bz); // Returns the magnetic field of the device - precise up to +/-1mT

    // Set Device Configuration Register Settings
    int8_t setCRCMode(uint8_t crcMode);     // Sets the I2C CRC byte to be sent
    int8_t setMagTemp(uint8_t magTempMode); // Selects the temp coefficient of the magnet
    int8_t setConvAvg(uint8_t avgMode);     // Sets additional sampling of the sensor data
    int8_t setIntThreshold(
        uint8_t threshold);               // Selects the threshold for the interrupt function (one direction, OR N/S)
    int8_t setReadMode(uint8_t readMode); // Sets the I2C read mode
    int8_t setLowPower(uint8_t lpLnMode); // Selects if device is operating in low power or low noise mode
    int8_t setGlitchFilter(uint8_t glitchMode); // Selects I2C filter on or off
    int8_t setTriggerMode(uint8_t trigMode);    // Sets condition that initiates a single conversion command
    int8_t setOperatingMode(uint8_t opMode);    // Selects operating mode and updates values based on operating mode
    int8_t setMagChannel(uint16_t channelMode); // Sets the data acquisition from magnetic axis channels
    int8_t setSleeptime(uint16_t sleepTime);    // Sets the time spent in low power mode between conversions
    int8_t setMagDir(uint8_t threshDir);        // Sets the direction of threshold check
    int8_t setMagGain(uint8_t gainAdjust);      // Sets the axis for magnitude gain correction value
    int8_t setAngleEN(uint8_t angleEnable);     // Sets the angle caluclation, mag gain, and offset corections
    int8_t setXYAxisRange(uint8_t xyAxisRange); // Sets the X and Y axes magnetic range from ±40mT or ±80mT
    int8_t setZAxisRange(uint8_t zAxisRange);   // Sets the Z axis magnetic range from ±40mT or ±80mT
    int8_t setXThresh(uint32_t xThresh);        // Sets the X axis threshold code for limit check
    int8_t setYThresh(uint32_t yThresh);        // Sets the Y axis threshold code for limit check
    int8_t setZThresh(uint32_t zThresh);        // Sets the Z axis threshold code for limit check
    int8_t setTempThresh(uint32_t tempThresh);  // Sets the Temperature threshold code for limit check
    int8_t setTempEN(
        bool temperatureEnable); // Sets the enable bit that determines the data acquisition of the temp channel
    int8_t setIntRslt(bool temperatureEnable); // Sets the enable interrupt response bit on conversion complete
    int8_t setThreshEn(
        bool enableInterruptResponse); // Configures bit to enable interrupt response on predefined thershold cross
    int8_t setIntState(bool interruptState);      // Emables the interrupt if latched or pulsed
    int8_t setIntMode(uint8_t configurationMode); // Configures the interrupt mode select
    int8_t setMaskInt(bool interruptPinEnable);   // Configres the Mask !INT pin when !INT connected to GND
    int8_t setI2CAddress(uint16_t address);       // Change these bits to a new I2C address if required
    int8_t setI2CAddressEN(
        bool addressEnable); // Enables/disables bit to allow the user to change the I2C address of the devices
    int8_t setOscillatorError(bool oscError); // Clear the Oscillator Error pin accordingly

    // Get Device Configuration Register Settings
    uint16_t getCRCMode();       // Returns the I2C CRC byte to be sent
    uint16_t getMagTemp();       // Returns the temperature coefficient of the magnet
    uint16_t getConvAvg();       // Returns the average sampling rate of the sensor data
    uint16_t getIntThreshold();  // Returns the threshold for the interrupt function
    uint16_t getLowPower();      // Returns if the device is operating in low power or noise mode
    uint16_t getGlitchFiler();   // Returns I2C glitch filter on or off
    uint16_t getTriggerMode();   // Returns if trigger is set to I2C command or INT pin
    uint16_t getOperatingMode(); // Returns the operating mode from 1 of the 4 modes.
    uint16_t getMagChannel();    // Returns data acquisiton from the list of mag axis channels
    uint16_t getSleeptime();     // Returns the time spent in low power mode
    uint16_t getMagDir();        // Returns the direction of threshold check
    uint16_t getMagGain();       // Returns the axis for magnitude gain correction value
    uint16_t getAngleEn();       // Returns the angle calculation and associated channel order
    uint16_t getXYAxisRange();   // Returns the X and Y axes magnetic range
    uint16_t getZAxisRange();    // Returns the Z axis magnetic range
    uint16_t getXThresh();       // Returns the X axis threshold code for limit check
    uint16_t getYThresh();       // Returns the Y axis threshold code for limit check
    uint16_t getZThresh();       // Returns the Z axis threshold code for limit check
    uint16_t getTempThresh();    // Returns the temperature threshold code entered by the user
    uint16_t getTempEN();        // Returns the enable bit to determine if temp channel is on or off
    uint16_t getIntRslt();       // Returns the enable interrupt response bit on conversion complete.
    uint16_t getThreshEn();      // Returns the bit that enables the interrupt
    uint16_t getIntState();      // Returns the !INT interrupt if latched or pulsed
    uint16_t getIntMode();       // Returns the configuration for the interrupt mode select
    uint16_t getMaskInt();       // Returns the Mask !INT pin when !INT is connected to GND
    uint16_t getSetCount();      // Returns the rolling count of conversion data sets
    uint16_t getPOR();           // Returns if the device is poweres up or expereinced POR
    uint16_t getDiagStatus();    // Returns if there was a detection of any internal diagnostics fail
    uint16_t getResultStatus();  // Returns the conversion data buffer status (Data complete or not)

    uint16_t getI2CAddress();     // Returns the I2C address of the device
    uint16_t getDeviceID();       // Returns the device version indicator
    uint16_t getManufacturerID(); // Returns the manufacturer ID

    float getTemp();            // Uses T_MSB_RESULT and T_LSB_RESULT Registers
    float getXData();           // Uses X_MSB_RESULT and X_LSB_RESULT Registers
    float getYData();           // Uses Y_MSB_RESULT and Y_LSB_RESULT Registers
    float getZData();           // Uses Z_MSB_RESULT and Z_LSB_RESULT Registers
    float getAngleResult();     // Uses ANGLE_RESULT_LSB and ANGLE_RESULT_MSB Registesr
    float getMagnitudeResult(); // Uses the MAGNITUDE_RESULT Register

    uint16_t getInterruptPinStatus(); // Returns the interrupt pin status
    uint16_t getOscillatorError();    // Returns the Oscillator Error Status
    uint16_t getIntPinError();        // Returns the Interrupt Pin Status
    uint16_t getOtpCrcError();        // Returns the OTP CRC Error Status
    uint16_t getUndervoltageError();  // Returns the undervoltage error status

    uint16_t getDeviceStatus(); // Returns the error detected

    int8_t getError(); // Returns an error code (0 is success, negative is failure, positive is warning)

  private:
    // I2C Communication interface settings
    TwoWire *_i2cPort = NULL;
    uint8_t _deviceAddress;

    uint16_t readRegister(uint8_t reg);             // Reads 2 register bytes from sensor
    void writeRegister(uint8_t reg, uint16_t data); // Wires single byte of data to the sensor

    void configI2C_RD(uint8_t read_mode);
};

#endif