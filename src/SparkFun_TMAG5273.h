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

#define DEVICE_ID_VALUE 0x5449   // Value found in the device ID register
#define I2C_ADDRESS_INITIAL 0X22 // Initial I2C address value - can be changed using functions as seen below
#define TSENSE_T0 25             // Reference temperature for TADC_T0
#define TADC_T0 17508            // Temp result in decimal value (from 16-buit format)
#define TADC_RES 60.1            // Temperature sensing resolution (in 16-bit format)

#define CRC_DISABLE 0X0 // Disables I2C CRC byte to be sent
#define CRC_ENABLE 0X1  // Enable I2C CRC byte to be sent

#define X1_CONVERSION 0X0  // 1X Average
#define X2_CONVERSION 0X1  // 2X Average
#define X4_CONVERSION 0X2  // 4X Average
#define X8_CONVERSION 0X3  // 8X Average
#define X16_CONVERSION 0X4 // 16X Average
#define X32_CONVERSION 0X5 // 32X Average

#define I2C_MODE_3BYTE 0X0       // Standard I2C 3-byte read command
#define I2C_MODE_1BYTE_16BIT 0X1 // 1-byte I2C read command for 16bit sensor data and conversion status
#define I2C_MODE_1BYTE_8BIT 0X2  // 1-byte I2C read command for 8 bit sensor MSB data and conversion status

#define LOW_ACTIVE_CURRENT_MODE 0X0 // Low active current mode
#define LOW_NOISE_MODE 0X1          // Low noise mode

#define GLITCH_ON 0X0  // Glitch filter on
#define GLITCH_OFF 0X1 // Glitch filter off

#define STANDY_BY_MODE 0X0          // Stand-by mode
#define SLEEP_MODE 0X1              // Sleep mode
#define CONTINUOUS_MEASURE_MODE 0X2 // Continous measure mode
#define WAKE_UP_AND_SLEEP_MODE 0X3  // Wake-up and sleep mode

#define CHANNELS_OFF 0X0     
#define X_ENABLE 0X1     // X Channel enabled
#define Y_ENABLE 0X2     // Y Channel enabled
#define X_Y_ENABLE 0X3   // X, Y Channel enabled
#define Z_ENABLE 0X4     // Z Channel enabled
#define Z_X_ENABLE 0X5   // Z, X Channel enabled
#define Y_Z_ENABLE 0X6   // Y, Z Channel enabled
#define X_Y_Z_ENABLE 0X7 // X, Y, Z Channel enabled
#define XYX_ENABLE 0X8   // XYX Channel enabled
#define YXY_ENABLE 0X9   // YXY Channel enabled
#define YZY_ENABLE 0XA   // YZY Channel enabled
#define XZX_ENABLE 0XB   // XZX Channel enabled

#define SLEEP_1MS 0X0     // 1ms
#define SLEEP_5MS 0X1     // 5ms
#define SLEEP_10MS 0X2    // 10 ms
#define SLEEP_15MS 0X3    // 15ms
#define SLEEP_20MS 0X4    // 20ms
#define SLEEP_30MS 0X5    // 30ms
#define SLEEP_50MS 0X6    // 50ms
#define SLEEP_100MS 0X7   // 100ms
#define SLEEP_500MS 0X8   // 500ms
#define SLEEP_1000MS 0X9  // 1000ms
#define SLEEP_2000MS 0XA  // 2000ms
#define SLEEP_5000MS 0XB  // 5000ms
#define SLEEP_20000MS 0XC // 20000ms

#define THRESHOLD_1 0X0 // 1 Threshold crossing
#define THRESHOLD_4 0X1 // 4 Threshold crossing

#define THRESHOLD_INT_ABOVE 0X0 // Sets interrupt for field above the threshold
#define THRESHOLD_INT_BELOW 0X1 // Sets interrupt for field below the threshold

#define GAIN_ADJUST_CHANNEL_1 0X0 // 1st channel is selected for gain adjustment
#define GAIN_ADJUST_CHANNEL_2 0X1 // 2nd channel is selected for gain adjustment

#define NO_ANGLE_CALCULATION 0X0 // No angle calculation, magnetic gain, and offset correction enabled
#define XY_ANGLE_CALCULATION 0X1 // X 1st, Y 2nd
#define YZ_ANGLE_CALCULATION 0X2 // Y 1st, Z 2nd
#define XZ_ANGLE_CALCULATION 0X3 // X 1st, Z 2nd

#define RANGE_40MT 0X0 // +/-40mT, DEFAULT
#define RANGE_80MT 0X1 // +/-80mT, DEFAULT

#define TEMPERATURE_DISABLE 0X0 // Temperature channel disabled
#define TEMPERATURE_ENABLE 0X1  // Temperature channel enabled

#define INTERRUPT_NOT_ASSERTED 0X0 // Interrupt is not asserved when set
#define INTERRUPT_ASSERTED 0X1     // Interrupt is asserted

#define NO_INTERRUPT 0X0              // No interrupt
#define INTERRUPT_THROUGH_INT 0X1     // Interrupt thru INT
#define INTERRUPT_THROUGH_INT_I2C 0X2 // Interrupt thru INT except when I2C bus is busy
#define INTERRUPT_THROUGH_SCL 0X3     // Interrupt thru SCL
#define INTERRUPT_THROUGH_SCL_I2C 0X4 // Interrupt thru SCL except when I2C bus is busy

#define INTERRUPT_ENABLED 0X0  // Interrupt pin is enabled
#define INTERRUPT_DISABLED 0X1 // Interrupt pin is disabled

#define INT_PIN_ENABLE 0X0  // INT pin is enabled
#define INT_PIN_DISABLE 0X1 // INT pin is disabled (for wake-up and trigger functions)

#define I2C_ADDRESS_CHANGE_DISABLE 0X0 // Disable update of I2C address
#define I2C_ADDRESS_CHANGE_ENABLE 0X1  // Enable update of I2C address

#define NO_POR 0X0  // Conversion data not complete
#define YES_POR 0X1 // Conversion data complete

#define OSCILLATOR_ERROR_UNDETECTED 0X0 // No Oscillator error detected
#define OSCILLATOR_ERROR_DETECTED 0X1   // Oscillator error detected

#define INT_ERROR_UNDETECTED 0X0 // No INT error detected
#define INT_ERROR_DETECTED 0X1   // INT error detected

#define DEVICE_CONFIG_1_BEGIN 0X1  // Value to set the DEVICE_CONFIG_1 register to on startup
#define SENSOR_CONFIG_1_BEGIN 0X79 // Value to set the SENSOR_CONFIG_1 register to on startup
#define T_CONFIG_BEGIN 0X1         // Value to set the T_CONFIG register to on startup
#define INT_CONFIG_1_BEGIN 0XA4    // Value to set the INT_CONFIG_1 register to on startup
#define DEVICE_CONFIG_2_BEGIN 0X02 // Value to set the DEVICE_CONFIG_2 register to on startup

#define TMAG_OK INT8_C(0) // To define success code

struct TMAG5273_Interface
{
};

class TMAG5273
{
  public:
    TMAG5273(); // Constructor

    // Return to PRIVATE after debugging
    uint8_t _deviceAddress;

    int8_t begin(uint8_t sensorAddress = 0X22,
                 TwoWire &wirePort =
                     Wire); // Checks for ACK over I2C, and sets the device ID of the TMAG and chooses the wire port
    int8_t setupWakeUpAndSleep(); // Sets the device to be in Wake Up and Sleep Mode

    int8_t readWakeUpAndSleepData(float *xVal, float *yVal, float *zVal,
                                  float *temperature); // Returns the data for the X, Y, Z, and T values

    // Set Device Configuration Register Settings
    int8_t setCRCMode(uint8_t crcMode);     // Sets the I2C CRC byte to be sent
    int8_t setMagTemp(uint8_t magTempMode); // Selects the temp coefficient of the magnet
    int8_t setConvAvg(uint8_t avgMode);     // Sets additional sampling of the sensor data
    int8_t setReadMode(uint8_t readMode);   // Sets the I2C read mode of the device
    int8_t setIntThreshold(
        uint8_t threshold);               // Selects the threshold for the interrupt function (one direction, OR N/S)
    int8_t setLowPower(uint8_t lpLnMode); // Selects if device is operating in low power or low noise mode
    int8_t setGlitchFilter(uint8_t glitchMode);     // Selects I2C filter on or off
    int8_t setTriggerMode(uint8_t trigMode);        // Sets condition that initiates a single conversion command
    int8_t setOperatingMode(uint8_t opMode);        // Selects operating mode and updates values based on operating mode
    int8_t setMagneticChannel(uint8_t channelMode); // Sets the data acquisition from magnetic axis channels
    int8_t setSleeptime(uint8_t sleepTime);         // Sets the time spent in low power mode between conversions
    int8_t setMagDir(uint8_t threshDir);            // Sets the direction of threshold check
    int8_t setMagnitudeGain(uint8_t gainAdjust);    // Sets the axis for magnitude gain correction value
    int8_t setMagneticGain(float magneticGain);     // Sets the 8-bit gain value to adjust a Hall axis gain
    int8_t setMagneticOffset1(float offset1);       // Sets the offset value determined by a primary for the first axis
    int8_t setMagneticOffset2(float offset2);       // Sets the offset value determined by a primary for the second axis
    int8_t setAngleEn(uint8_t angleEnable);         // Sets the angle caluclation, mag gain, and offset corections
    int8_t setXYAxisRange(uint8_t xyAxisRange);     // Sets the X and Y axes magnetic range from ±40mT or ±80mT
    int8_t setZAxisRange(uint8_t zAxisRange);       // Sets the Z axis magnetic range from ±40mT or ±80mT
    int8_t setXThreshold(float xThreshold);         // Sets the X axis threshold code for limit check
    int8_t setYThreshold(float yThreshold);         // Sets the Y axis threshold code for limit check
    int8_t setZThreshold(float zThresh);            // Sets the Z axis threshold code for limit check
    int8_t setTemperatureThreshold(float tempThresh); // Sets the Temperature threshold code for limit check
    int8_t setTemperatureEn(
        bool temperatureEnable); // Sets the enable bit that determines the data acquisition of the temp channel
    int8_t setInterruptResult(bool interruptEnable); // Sets the enable interrupt response bit on conversion complete
    int8_t setThresholdEn(
        bool enableInterruptResponse); // Configures bit to enable interrupt response on predefined thershold cross
    int8_t setIntPinState(bool interruptState);         // Emables the interrupt if latched or pulsed
    int8_t setInterruptMode(uint8_t configurationMode); // Configures the interrupt mode select
    int8_t setMaskInterrupt(bool interruptPinEnable);   // Configres the Mask !INT pin when !INT connected to GND
    int8_t setI2CAddress(uint8_t address);              // Change these bits to a new I2C address if required
    int8_t setI2CAddressEN(
        bool addressEnable); // Enables/disables bit to allow the user to change the I2C address of the devices
    int8_t setOscillatorError(bool oscError); // Clear the Oscillator Error pin accordingly

    // Get Device Configuration Register Settings
    uint8_t getCRCMode();                // Returns the I2C CRC byte to be sent
    uint8_t getMagTemp();                // Returns the temperature coefficient of the magnet
    uint8_t getConvAvg();                // Returns the average sampling rate of the sensor data
    uint8_t getReadMode();               // Returns the I2C read mode
    uint8_t getIntThreshold();           // Returns the threshold for the interrupt function
    uint8_t getLowPower();               // Returns if the device is operating in low power or noise mode
    uint8_t getGlitchFiler();            // Returns I2C glitch filter on or off
    uint8_t getTriggerMode();            // Returns if trigger is set to I2C command or INT pin
    uint8_t getOperatingMode();          // Returns the operating mode from 1 of the 4 modes.
    uint8_t getMagneticChannel();        // Returns data acquisiton from the list of mag axis channels
    uint8_t getSleeptime();              // Returns the time spent in low power mode
    uint8_t getMagDir();                 // Returns the direction of threshold check
    uint8_t getMagnitudeChannelSelect(); // Returns the axis for magnitude gain correct XYAxisRange section value
    uint8_t getMagneticGain();           // Returns the value determined to adjust a Hall axis gain
    int8_t getMagneticOffset1();         // Returns the offset value determined by a primary for the first axis
    int8_t getMagneticOffset2();         // Returns the offset value determined by a primary for the second axis
    uint8_t getAngleEn();                // Returns the angle calculation and associated channel order
    uint8_t getXYAxisRange();            // Returns the X and Y axes magnetic range
    uint8_t getZAxisRange();             // Returns the Z axis magnetic range
    float getXThreshold();               // Returns the X axis threshold code for limit check
    float getYThreshold();               // Returns the Y axis threshold code for limit check
    float getZThreshold();               // Returns the Z axis threshold code for limit check
    float getTemperatureThreshold();     // Returns the temperature threshold code entered by the user
    uint8_t getTemperatureEN();          // Returns the enable bit to determine if temp channel is on or off
    uint8_t getInterruptResult();        // Returns the enable interrupt response bit on conversion complete.
    uint8_t getThresholdEn();            // Returns the bit that enables the interrupt
    uint8_t getIntPinState();            // Returns the !INT interrupt if latched or pulsed
    uint8_t getInterruptMode();          // Returns the configuration for the interrupt mode select
    uint8_t getMaskInt();                // Returns the Mask !INT pin when !INT is connected to GND
    uint8_t getSetCount();               // Returns the rolling count of conversion data sets
    uint8_t getPOR();                    // Returns if the device is powered up or expereinced POR
    uint8_t getDiagStatus();             // Returns if there was a detection of any internal diagnostics fail
    uint8_t getResultStatus();           // Returns the conversion data buffer status (Data complete or not)
    uint8_t getI2CAddress();             // Returns the I2C address of the device
    uint8_t getDeviceID();               // Returns the device version indicator
    uint16_t getManufacturerID();        // Returns the manufacturer ID

    float getTemp();            // Uses T_MSB_RESULT and T_LSB_RESULT Registers
    float getXData();           // Uses X_MSB_RESULT and X_LSB_RESULT Registers
    float getYData();           // Uses Y_MSB_RESULT and Y_LSB_RESULT Registers
    float getZData();           // Uses Z_MSB_RESULT and Z_LSB_RESULT Registers
    float getAngleResult();     // Uses ANGLE_RESULT_LSB and ANGLE_RESULT_MSB Registers
    float getMagnitudeResult(); // Uses the MAGNITUDE_RESULT Register

    uint8_t getInterruptPinStatus(); // Returns the interrupt pin status
    uint8_t getOscillatorError();    // Returns the Oscillator Error Status
    uint8_t getIntPinError();        // Returns the Interrupt Pin Status
    uint8_t getOtpCrcError();        // Returns the OTP CRC Error Status
    uint8_t getUndervoltageError();  // Returns the undervoltage error status

    uint8_t getDeviceStatus(); // Returns the error detected

    int8_t getError(); // Returns an error code (0 is success, negative is failure, positive is warning)

  private:
    // I2C Communication interface settings
    TwoWire *_i2cPort = NULL;

    // uint8_t readRegister(uint8_t reg);                // Reads 2 register bytes from sensor
    // uint8_t writeRegister(uint8_t reg, uint8_t data); // Wires single byte of data to the sensor

    int8_t writeRegisters(uint8_t regAddress, uint8_t *dataBuffer, uint8_t numBytes);
    int8_t readRegisters(uint8_t regAddress, uint8_t *dataBuffer, uint8_t numBytes);
    uint8_t readRegister(uint8_t regAddress);
    uint8_t writeRegister(uint8_t regAddress, uint8_t data);
    bool ping(uint8_t i2c_address); // Checks for device presence
};

#endif