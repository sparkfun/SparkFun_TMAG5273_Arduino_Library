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
    TMAG5273 Breakout Version: 1.0.1
Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_TMAG5273_Arduino_Library.h"
#include <Arduino.h>
#include <Wire.h>

// CTOR
TMAG5273::TMAG5273()
{
    /* Nothing to do */
}

/// @brief Begin communication with the TMAG over I2C, initialize it, and
/// and set the wire for the I2C communication
/// @param sensorAddress I2C address of the sensor
/// @param wirePort I2C port to use for communication, defaults to Wire
/// @return Error code (1 is success, 0 is failure, negative is warning)
int8_t TMAG5273::begin(uint8_t sensorAddress, TwoWire &wirePort)
{

    // Setup the toolkit I2C port
    if (_theI2CBus.init(wirePort, sensorAddress) != ksfTkErrOk)
        return 0;

    // Makes sure the TMAG will acknowledge over I2C along with matching Device ID's
    if (isConnected() != 0)
        return 0;

    // Following the Detailed Design Procedure on page 42 of the datasheet
    setMagneticChannel(TMAG5273_X_Y_Z_ENABLE);
    setTemperatureEn(true);
    setOperatingMode(TMAG5273_CONTINUOUS_MEASURE_MODE);

    // 12/2025 - add as part of update -- from a PR pending on the repo
    // When the device is reset, these settings must also be reset or init can fail

    // No angle calculation
    setAngleEn(TMAG5273_NO_ANGLE_CALCULATION);

    // Set to low active current mode
    setLowPower(TMAG5273_LOW_ACTIVE_CURRENT_MODE);

    // Set the axis ranges for the device to be the largest
    setXYAxisRange(TMAG5273_RANGE_80MT);
    setZAxisRange(TMAG5273_RANGE_80MT);

    // Check if there is any issue with the device status register
    if (getError() != 0)
        return 0;

    // Check the low active current mode (0)
    if (getLowPower() != TMAG5273_LOW_ACTIVE_CURRENT_MODE)
        return 0;

    // Check the operating mode to make sure it is set to continuous measure (0X2)
    if (getOperatingMode() != TMAG5273_CONTINUOUS_MEASURE_MODE)
        return 0;

    // Check that all magnetic channels have been enables(0X7)
    if (getMagneticChannel() != TMAG5273_X_Y_Z_ENABLE)
        return 0;

    // Check that the temperature data acquisition has been enabled
    if (getTemperatureEN() != TMAG5273_TEMPERATURE_ENABLE)
        return 0;

    // Check that X and Y angle calculation is disabled
    if (getAngleEn() != TMAG5273_NO_ANGLE_CALCULATION)
        return 0;

    // returns true if all the checks pass
    return 1;
}

/// @brief This function will make sure the TMAG5273 acknowledges
///  over I2C, along with checking the Device ID to ensure proper
///  connection.
/// @return Error code (0 is success, negative is failure)
int8_t TMAG5273::isConnected()
{
    // Ping the I2C bus to make sure it's connected
    if (_theI2CBus.ping() != ksfTkErrOk)
        return -1;

    if (getManufacturerID() != TMAG5273_DEVICE_ID_VALUE)
        return -1;

    return 0;
}

/// @brief Write to the correct registers the correct values to
///  enter wake up and sleep mode. See page 42 of the datasheet for more
///  information on the values chosen.
///     TMAG5273_REG_INT_CONFIG_1
///     TMAG5273_REG_DEVICE_CONFIG_2
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setupWakeUpAndSleep()
{
    sfTkError_t rc =
        _theI2CBus.writeRegister(TMAG5273_REG_INT_CONFIG_1, TMAG5273_WAKEUP_SLEEP_MODE_INT_CONFIG_1_SETTINGS);
    if (rc != ksfTkErrOk)
        return -1;

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, TMAG5273_WAKEUP_SLEEP_MODE_DEVICE_CONFIG_2_SETTINGS);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Read the 16-bit magnetic reading for the X, Y, Z and temperature
///  data during the wakeup and sleep mode
/// @param xVal 16-bit value for the X magnetic reading
/// @param yVal 16-bit value for the Y magnetic reading
/// @param zVal 16-bit value for the Z magnetic reading
/// @param temperature 8-bit value for the temperature reading
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::readWakeUpAndSleepData(float &xVal, float &yVal, float &zVal, float &temperature)
{

    xVal = getXData();
    yVal = getYData();
    zVal = getZData();
    temperature = getTemp();

    return getError();
}

/************************************************************************************************/
/**************************        SET CONFIGURATION SETTINGS         ***************************/
/************************************************************************************************/

/// @brief Sets the I2C CRC byte to be sent
/// @param crcMode 0b0-0b1
/// 0X0 = CRC disabled
/// 0X1 = CRC enabled
/// TMAG5273_REG_DEVICE_CONFIG_1 - bit 7
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setCRCMode(uint8_t crcMode)
{

    if (crcMode > TMAG5273_CRC_ENABLE) // only 0x0 and 0x1 are valid
        return -1;                     // invalid crcMode

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, crcMode, TMAG5273_CRC_MODE_BITS, TMAG5273_CRC_MODE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;
    return getError();
}

/// @brief Sets the temperature coefficient of the magnet
/// @param magTempMode value to set the temp coefficient of the device
///     0x0 = 0% (No temperature compensation)
///     0x1 = 0.12%/deg C (NdBFe)
///     0x2 = Reserved
///     0x3 = 0.2%/deg C (Ceramic)
///     TMAG5273_REG_DEVICE_CONFIG_1 - bit 6-5
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagTemp(uint8_t magTempMode)
{
    if (magTempMode > TMAG5273_MAG_TEMP_0P2PCT ||
        magTempMode == TMAG5273_MAG_TEMP_RESERVED) // only 0x0 to 0x3 are valid
        return -1;
    // invalid magTempMode
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, magTempMode, TMAG5273_MAG_TEMP_BITS, TMAG5273_MAG_TEMP_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;
    return getError();
}

/// @brief Sets the additional sampling of the sensor data to reduce the
/// noise effect (or to increase resolution)
/// @param avgMode value to set the conversion average
///     0X0 = 1x average, 10.0-kSPS (3-axes) or 20-kSPS (1 axis)
///     0X1 = 2x average, 5.7-kSPS (3-axes) or 13.3-kSPS (1 axis)
///     0X2 = 4x average, 3.1-kSPS (3-axes) or 8.0-kSPS (1 axis)
///     0X3 = 8x average, 1.6-kSPS (3-axes) or 4.4-kSPS (1 axis)
///     0X4 = 16x average, 0.8-kSPS (3-axes) or 2.4-kSPS (1 axis)
///     0X5 =  32x average, 0.4-kSPS (3-axes) or 1.2-kSPS (1 axis)
///     TMAG5273_REG_DEVICE_CONFIG_1 - bit 4-2
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setConvAvg(uint8_t avgMode)
{
    if (avgMode > TMAG5273_X32_CONVERSION) // only 0x0 to 0x5 are valid
        return -1;                         // invalid avgMode

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, avgMode, TMAG5273_CONV_AVG_BITS, TMAG5273_CONV_AVG_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Defines the I2C read mode of the device
/// @param readMode Value to set the read mode
///     0X0 = Standard I2C 3-byte read command
///     0X1 = 1-byte I2C read command for 16bit sensor data and
///           conversion status
///     0X2 = 1-byte I2C read command for 8bit sensor MSB data and
///           conversion status
///     TMAG5273_REG_DEVICE_CONFIG_1 Bits 0-1
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setReadMode(uint8_t readMode)
{
    if (readMode > TMAG5273_I2C_MODE_1BYTE_8BIT) // only 0x0 to 0x2 are valid
        return -1;                               // invalid readMode

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, readMode, TMAG5273_I2C_READ_MODE_BITS, TMAG5273_I2C_READ_MODE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the threshold for the interrupt function
/// @param threshold Value to set the threshold
///     0X0 = Takes the 2's complement value of each x_THR_CONFIG
///       register to create a magnetic threshold of the corresponding axis
///     0X1 = Takes the 7LSB bits of the x_THR_CONFIG register to create
///       two opposite magnetic thresholds (one north, and another south)
///       of equal magnitude
///     The rest of the bits are reserved (0x2 - 0x7) - DO NOT USE
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 7-5
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setIntThreshold(uint8_t threshold)
{
    if (threshold > TMAG5273_THRESHOLD_HYST_7LSB) // only 0x0 and 0x1 are valid
        return -1;                                // invalid threshold

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, threshold, TMAG5273_THR_HYST_BITS, TMAG5273_THR_HYST_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the device to low power or low noise mode
/// @param lpLnMode Value to set the mode
///    0X0 = Low active current mode
///    0X1 = Low noise mode
///    TMAG5273_REG_DEVICE_CONFIG_2 - bit 4
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setLowPower(uint8_t lpLnMode)
{
    if (lpLnMode > TMAG5273_LOW_NOISE_MODE) // only 0x0 and 0x1 are valid
        return -1;                          // invalid lpLnMode

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, lpLnMode, TMAG5273_LOW_POWER_BITS, TMAG5273_LOW_POWER_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the I2C glitch filter on and off
/// @param glitchMode value to set the mode
///     0x0 = Glitch filter ON
///     0X0 = Glitch filter OFF
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 3
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setGlitchFilter(uint8_t glitchMode)
{
    if (glitchMode > TMAG5273_GLITCH_OFF) // only 0x0 and 0x1 are valid
        return -1;                        // invalid glitchMode

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, glitchMode, TMAG5273_GLITCH_FILTER_BITS, TMAG5273_GLITCH_FILTER_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets a condition which initiates a single conversion based off
///  already configured registers. A running conversion completes before
///  executing a trigger. Redundant triggers are ignored. TRIGGER_MODE
///  is available only during the mode 'Stand-by mode' mentioned in OPERATING_MODE.
/// @param trigMode value to set the trigger mode of the device
///     0X0 = Conversion Start at I2C Command Bits, DEFAULT
///     0X1 = Conversion starts through trigger signal at !INT pin
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 2
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setTriggerMode(uint8_t trigMode)
{
    if (trigMode > TMAG5273_TRIGGER_INT_SIGNAL) // only 0x0 and 0x1 are valid
        return -1;                              // invalid trigMode

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, trigMode, TMAG5273_TRIGGER_MODE_BITS, TMAG5273_TRIGGER_MODE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the operating mode from one of the 4 modes:
///  stand-by mode, sleep mode, continuous measure mode, and
///  wake-up and sleep mode.
/// @param opMode value to set the operating mode of the device
///     0X0 = Stand-by mode (starts new conversion at trigger event)
///     0X1 = Sleep mode
///     0X2 = Continuous measure mode
///     0X3 = Wake-up and sleep mode (W&S Mode)
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 1-0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setOperatingMode(uint8_t opMode)
{
    if (opMode > TMAG5273_WAKE_UP_AND_SLEEP_MODE) // only 0x0 to 0x3 are valid
        return -1;                                // invalid opMode

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, opMode, TMAG5273_OPERATING_MODE_BITS, TMAG5273_OPERATING_MODE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the data acquisition from the following magnetic
///  axis channels listed below
/// @param channelMode Value that sets the channel for data acquisition
///     0X0 = All magnetic channels off, DEFAULT
///     0X1 = X Channel Enabled
///     0X2 = Y Channel Enabled
///     0X3 = X, Y Channel Enabled
///     0X4 = Z Channel Enabled
///     0X5 = Z, X Channel Enabled
///     0X6 = Y, Z Channel Enabled
///     0X7 = X, Y, Z Channel Enabled
///     0X8 = XYX Channel Enabled
///     0X9 = YXY Channel Enabled
///     0XA = YZY Channel Enabled
///     0XB = XZX Channel Enabled
///     TMAG5273_REG_SENSOR_CONFIG_1 - bits 7-4
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagneticChannel(uint8_t channelMode)
{
    // validate the channelMode input ( valid <= TMAG5273_XZX_ENABLE (0xB) )
    if (channelMode > TMAG5273_XZX_ENABLE)
        return -1; // invalid channelMode

    uint8_t mode = 0;

    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, channelMode, TMAG5273_CHANNEL_MODE_BITS, TMAG5273_CHANNEL_MODE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the time spent in low power mode between conversions
///  when OPERATING_MODE = 11b.
/// @param sleepTime Value to set the time desired
///     0X0 = 1ms
///     0X1 = 5ms
///     0X2 = 10ms
///     0X3 = 15ms
///     0X4 = 20ms
///     0X5 = 30ms
///     0X6 = 50ms
///     0X7 = 100ms
///     0X8 = 500ms
///     0X9 = 1000ms
///     0XA = 2000ms
///     0XB = 5000ms
///     0XC = 20000ms
///     TMAG5273_REG_SENSOR_CONFIG_1 - bit 3-0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setSleeptime(uint8_t sleepTime)
{
    // validate the sleepTime input ( valid <= TMAG5273_SLEEP_20000MS (0xC) )
    if (sleepTime > TMAG5273_SLEEP_20000MS)
        return -1; // invalid sleepTime

    uint8_t mode = 0;

    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, sleepTime, TMAG5273_SLEEP_MODE_BITS, TMAG5273_SLEEP_MODE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the direction of threshold check. This bit
///  is ignored when THR_HYST > 001b
/// @param threshDir value to set the direction of threshold
///     0X0 = sets interrupt for field above the threshold
///     0X1 = sets interrupt for field below the threshold
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 5
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagDir(uint8_t threshDir)
{
    if (threshDir > TMAG5273_THRESHOLD_INT_BELOW) // only 0x0 and 0x1 are valid
        return -1;                                // invalid threshDir

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, threshDir, TMAG5273_THRESHOLD_INT_BITS, TMAG5273_THRESHOLD_INT_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the axis for magnitude gain correction valued
/// entered in MAG_GAIN_CONFIG register
/// @param gainAdjust Value to set the gain correction value
///     0X0 = 1st channel is selected for gain adjustment
///     0X1 = 2nd channel is selected for gain adjustment
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 4
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagnitudeGain(uint8_t gainAdjust)
{
    if (gainAdjust > TMAG5273_GAIN_ADJUST_CHANNEL_2) // only 0x0 and 0x1 are valid
        return -1;
    // invalid threshDir
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, gainAdjust, TMAG5273_GAIN_ADJUST_BITS, TMAG5273_GAIN_ADJUST_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief This function sets an 8-bit gain value determined by a
///  primary to adjust a Hall axis gain. The particular axis is selected
///  based off the settings of MAG_GAIN_CH and ANGLE_EN register bits.
///  The binary 8-bit input is interpreted as a fractional value between
///  0 and 1 based off the formula 'user entered value in decimal/256.'
///  Gain value of 0 is interpreted by the device as 1.
///     TMAG5273_REG_MAG_GAIN_CONFIG
/// @param magneticGain
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagneticGain(uint8_t magneticGain)
{

    // Convert magneticGain to be the correct value to write to their register

    // get our fractional value between 0 and 1
    float fMagGain = (float)magneticGain / 256.f;
    uint32_t iMagGain = 0;

    // copy over the float bits into an integer
    memcpy(&iMagGain, &fMagGain, sizeof(fMagGain));

    // IEE754 format has the fraction as 23 bits - bit 22-0 - so shift right 15 and use those bits
    uint8_t uMagGainFrac = (iMagGain >> 15) & 0xFF;

    if (_theI2CBus.writeRegister(TMAG5273_REG_MAG_GAIN_CONFIG, uMagGainFrac) != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief This function will write an 8-bit, 2's complement offset value
///  determined by a primary to adjust the first axis offset value. The
///  range of possible offset valid entrees can be +/-128. The offset value
///  is calculated by multiplying bit resolution with the entered value.
///     TMAG5273_REG_MAG_OFFSET_CONFIG_1
/// @param offset1 Value within the range +/-128 in mT
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagneticOffset1(int8_t offset1)
{
    if (offset1 < -128.f || offset1 > 128.f)
        return -1; // invalid offset1

    uint8_t rangeValXY = getXYAxisRange();
    float range = rangeValXY == 0 ? 40.f : 80.f;

    // From the datasheet, equation to calculate offset value (17)
    //
    // OFFSET_REG = (2^12 * OFFSET_mT) / 2*|RANGE_mT|
    // Where RANGE_mT is the full scale range of the selected axis
    // OFFSET_mT is the desired offset in mT - passed to this method
    //
    // factoring out the 2 in the denominator gives us:
    // OFFSET_REG = (2^11 * OFFSET_mT) / |RANGE_mT|
    int8_t magOffset = (int8_t)((2048.f * (float)offset1) / range); // 2048 = 2^11

    // deal with int/uint conversion
    uint8_t tempWrite = *(uint8_t *)&magOffset;
    if (_theI2CBus.writeRegister(TMAG5273_REG_MAG_OFFSET_CONFIG_1, tempWrite) != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief This function will write an 8-bit, 2's complement offset value
///  determined by a primary to adjust the first axis offset value. The
///  range of possible offset valid entrees can be +/-128. The offset value
///  is calculated by multiplying bit resolution with the entered value.
///     TMAG5273_REG_MAG_OFFSET_CONFIG_2
/// @param offset2 Value within the range +/-128
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagneticOffset2(int8_t offset2)
{
    if (offset2 < -128.f || offset2 > 128.f)
        return -1; // invalid offset2

    // Determine whether the magnetic offset channel is Y or Z for the range selection
    uint8_t channelSelect = getAngleEn(); // 1, 2, or 3
    if (channelSelect < 1 || channelSelect > 3)
        return -1; // invalid channel selection

    // See which XY axis range is currently set

    float range;
    if (channelSelect == 1)
        range = getXYAxisRange() == 0 ? 40.f : 80.f;
    else
        range = getZAxisRange() == 0 ? 40.f : 80.f;

    // From the datasheet, equation to calculate offset value (17)
    //
    // OFFSET_REG = (2^12 * OFFSET_mT) / 2*|RANGE_mT|
    // Where RANGE_mT is the full scale range of the selected axis
    // OFFSET_mT is the desired offset in mT - passed to this method
    //
    // factoring out the 2 in the denominator gives us:
    // OFFSET_REG = (2^11 * OFFSET_mT) / |RANGE_mT|
    int8_t magOffset = (int8_t)((2048.f * (float)offset2) / range); // 2048 = 2^11

    // deal with int/uint conversion
    uint8_t tempWrite = *(uint8_t *)&magOffset;

    // Write correct value to register
    if (_theI2CBus.writeRegister(TMAG5273_REG_MAG_OFFSET_CONFIG_2, tempWrite) != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the angle calculation, magnetic gain, and offset corrections
///  between two selected magnetic channels
/// @param angleEnable value to write to the register for which angle calculation enabled
///     0X0 = No angle calculation, magnitude gain, and offset
///            correction enabled
///     0X1 = X 1st, Y 2nd
///     0X2 = Y 1st, Z 2nd
///     0X3 = X 1st, Z 2nd
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 3-2
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setAngleEn(uint8_t angleEnable)
{
    if (angleEnable > TMAG5273_XZ_ANGLE_CALCULATION) // only 0x0 to 0x3 are valid
        return -1;                                   // invalid angleEnable

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, angleEnable, TMAG5273_ANGLE_CALCULATION_BITS, TMAG5273_ANGLE_CALCULATION_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the X and Y axes magnetic range from 2 different options
/// @param xyAxisRange Value to choose the magnetic range
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 1
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setXYAxisRange(uint8_t xyAxisRange)
{
    if (xyAxisRange > TMAG5273_RANGE_80MT) // only 0x0 and 0x1 are valid
        return -1;                         // invalid xyAxisRange

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, xyAxisRange, TMAG5273_XY_RANGE_BITS, TMAG5273_XY_RANGE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the Z magnetic range from 2 different options
/// @param zAxisRange Value to set the range from either 40mT or 80mT
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setZAxisRange(uint8_t zAxisRange)
{
    if (zAxisRange > TMAG5273_RANGE_80MT) // only 0x0 and 0x1 are valid
        return -1;                        // invalid zAxisRange

    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, zAxisRange, TMAG5273_Z_RANGE_BITS, TMAG5273_Z_RANGE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets an 8-bit, 2's complement X axis threshold code for limit
///  check. The range of possible threshold entrees can be +/-128. The
///  threshold value in mT is calculated as (40(1+X_Y_RANGE)/128)*X_THR_CONFIG,
///  where X_THR_CONFIG is the value set by the user.
///  Default 0h means no threshold comparison.
/// @param xThreshold 8-bit value to set the threshold for the X limit
///     TMAG5273_REG_X_THR_CONFIG - bits 7-0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setXThreshold(int8_t xThreshold)
{
    // deal with int/uint conversion
    uint8_t tempWrite = *(uint8_t *)&xThreshold;
    // Write the correct value to the register
    if (_theI2CBus.writeRegister(TMAG5273_REG_X_THR_CONFIG, tempWrite) != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets an 8-bit, 2's complement Y axis threshold code for limit
///  check. The range of possible threshold entrees can be +/-128. The
///  threshold value in mT is calculated as (40(1+X_Y_RANGE)/128)*Y_THR_CONFIG,
/// where Y_THR_CONFIG is the value set by the user.
///  Default 0h means no threshold comparison.
/// @param yThreshold 8-bit value to set the threshold for the Y limit
///     TMAG5273_REG_Y_THR_CONFIG - bits 7-0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setYThreshold(int8_t yThreshold)
{
    // deal with int/uint conversion
    uint8_t tempWrite = *(uint8_t *)&yThreshold;
    // Write the correct value to the register
    if (_theI2CBus.writeRegister(TMAG5273_REG_Y_THR_CONFIG, tempWrite) != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets an 8-bit, 2's complement Z axis threshold code for limit
///  check. The range of possible threshold entrees can be +/-128. The
///  threshold value in mT is calculated as (40(1+Z_RANGE)/128)*Z_THR_CONFIG,
///  where Z_THR_CONFIG is the value set by the user.
///  Default 0h means no threshold comparison.
/// @param zThresh 8-bit value to set the threshold for the Y limit
///     TMAG5273_REG_Z_THR_CONFIG - bits 7-0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setZThreshold(int8_t zThreshold)
{
    // deal with int/uint conversion
    uint8_t tempWrite = *(uint8_t *)&zThreshold;

    // Write the correct value to the register
    if (_theI2CBus.writeRegister(TMAG5273_REG_Z_THR_CONFIG, tempWrite) != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the temperature threshold code entered by the user. The
///  valid temperature threshold ranges are -41C to 170C with the threshold codes
///  for -41C = 1Ah, and 170C = 34h. Resolution is 8 degree C/ LSB
///  Default 0x0 means no threshold comparison.
/// @param tempThresh 8-bit value to set the threshold for the temperature limit
///     TMAG5273_REG_T_CONFIG - bits 7-1
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setTemperatureThreshold(int8_t tempThresh)
{
    // Write the new address into the register and enable the temperature to be read
    // NOTE: Temperature is NOT set -- this is not part of this function
    uint8_t tmp = (tempThresh << 1);
    if (_theI2CBus.writeRegister(TMAG5273_REG_T_CONFIG, tmp) != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the enable bit that determines the data acquisition of the
///  temperature channel.
/// @param temperatureEnable Value to determine enable or disable
///     0x0 = Temp Channel Disabled
///     0x1 = Temp Channel Enabled
///     TMAG5273_REG_T_CONFIG - bit 0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setTemperatureEn(bool temperatureEnable)
{
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_T_CONFIG, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, (uint8_t)temperatureEnable, TMAG5273_TEMPERATURE_BITS, TMAG5273_TEMPERATURE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_T_CONFIG, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Sets the enable interrupt response bit on conversion complete.
/// @param interruptEnable Value to determine if interrupt is or is not asserted
///     0X0 = Interrupt is NOT asserted when the configured set of
///           conversions are complete
///     0X1 = Interrupt is asserted when the configured set of
///           conversions are complete
///     TMAG5273_REG_INT_CONFIG_1 - bit 7
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setInterruptResult(bool interruptEnable)
{
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode =
        setBitFieldValue(mode, (uint8_t)interruptEnable, TMAG5273_INTERRUPT_RESULT_BITS, TMAG5273_INTERRUPT_RESULT_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Configures the bit that enables interrupt response on a
///   predefined threshold cross.
/// @param enableInterruptResponse Value to determine if interrupt is or is not asserted
///     0X0 = Interrupt is NOT asserted when a threshold is crossed
///     0X1 = Interrupt is asserted when a threshold is crossed
///     TMAG5273_REG_INT_CONFIG_1 - bit 6
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setThresholdEn(bool enableInterruptResponse)
{
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, (uint8_t)enableInterruptResponse, TMAG5273_INTERRUPT_THRESHOLD_BITS,
                            TMAG5273_INTERRUPT_THRESHOLD_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Enables the !INT interrupt if it is latched or pulsed.
/// @param interruptState Value to determine the interrupt state
///     0X0 = !INT interrupt latched until clear by a primary
///           addressing the device
///     0X1 = !INT interrupt pulse for 10us
///     TMAG5273_REG_INT_CONFIG_1 - bit 5
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setIntPinState(bool interruptState)
{
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, (uint8_t)interruptState, TMAG5273_INTERRUPT_PIN_STATE_BITS,
                            TMAG5273_INTERRUPT_PIN_STATE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Configures the interrupt mode select
/// @param config_mode Value to determine the int select
///     0X0 = No interrupt
///     0X1 = Interrupt through !INT
///     0X2 = Interrupt through !INT except when I2C bus is busy
///     0X3 = Interrupt through SCL
///     0X4 = Interrupt through SCL except when I2C bus is busy
///     TMAG5273_REG_INT_CONFIG_1 - bit 4-2
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setInterruptMode(uint8_t configurationMode)
{
    if (configurationMode > TMAG5273_INTERRUPT_THROUGH_SCL_I2C) // only 0x0 to 0x4 are valid
        return -1;
    // invalid configurationMode
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode = setBitFieldValue(mode, configurationMode, TMAG5273_INTERRUPT_MODE_BITS, TMAG5273_INTERRUPT_MODE_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief Configures the Mask !INT pin when !INT is connected to GND
/// @param interruptPinEnable Value to choose the INT enable or disable
///     0X0 = !INT pin is enabled
///     0X1 = !INT pin is disabled (for wake-up and trigger functions)
///     TMAG5273_REG_INT_CONFIG_1 - bit 0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMaskInterrupt(bool interruptPinEnable)
{
    uint8_t mode = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    mode =
        setBitFieldValue(mode, (uint8_t)interruptPinEnable, TMAG5273_INTERRUPT_MASK_BITS, TMAG5273_INTERRUPT_MASK_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_INT_CONFIG_1, mode);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief This register is loaded with the default I2C address
///  from OTP during first power up. Change these bits to a
///  new setting if a new I2C address is required (at each
///  power cycle these bits must be written again to avoid
///  going back to default factory address).
///  *NOTE*: To be able to use this function, make sure to
///  enable the I2C Address Bit (see function setI2CAddressEN
///  for more information).
/// @param address Address value to set device to
///     TMAG5273_REG_I2C_ADDRESS - bits 7-1
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setI2CAddress(uint8_t address)
{
    // Write the new address into the register and enable the register to be read
    uint8_t validAddress = (address << 1) | 0x01; // Make sure address is 7 bits
    if (_theI2CBus.writeRegister(TMAG5273_REG_I2C_ADDRESS, validAddress) != ksfTkErrOk)
        return -1;
    // Set the device address to the new address chosen
    _theI2CBus.setAddress(address);

    return getError();
}

/// @brief Writes to the I2C_ADDRESS_UPDATE_EN bit to enable
///  a new user defined I2C address.
/// @param addressEnable Value to determine if the user can write a new address.
///     0X0 = Disable update of I2C address
///     0X1 = Enable update of I2C address with bits (7:1)
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setI2CAddressEN(bool addressEnable)
{
    uint8_t addReg = 0;
    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_I2C_ADDRESS, addReg);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    addReg = setBitFieldValue(addReg, (uint8_t)addressEnable, TMAG5273_I2C_ADDRESS_BITS, TMAG5273_I2C_ADDRESS_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_I2C_ADDRESS, addReg);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/// @brief This function clears the oscillator error flag
///  when it is raised high by an error.
/// @param oscError Write '1' to clear the error flag
///     0X0 = No oscillator error detected
///     0X1 = Oscillator error detected
///     TMAG5273_REG_DEVICE_STATUS - bit 3
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setOscillatorError(bool oscError)
{

    uint8_t deviceStatusReg = 0;

    sfTkError_t rc = _theI2CBus.readRegister(TMAG5273_REG_DEVICE_STATUS, deviceStatusReg);
    if (rc != ksfTkErrOk)
        return -1;

    // clear out the target bits, shift data to location and or in new value
    deviceStatusReg = setBitFieldValue(deviceStatusReg, (uint8_t)oscError, TMAG5273_OSCILLATOR_ERROR_BITS,
                                       TMAG5273_OSCILLATOR_ERROR_LSB);

    rc = _theI2CBus.writeRegister(TMAG5273_REG_DEVICE_STATUS, deviceStatusReg);
    if (rc != ksfTkErrOk)
        return -1;

    return getError();
}

/************************************************************************************************/
/**************************        GET CONFIGURATION SETTINGS         ***************************/
/************************************************************************************************/

/// @brief Returns the I2C CRC byte to be sent
///     0X0 = CRC disabled
///     0X1 = CRC enabled
///     TMAG5273_REG_DEVICE_CONFIG_1 - bit 7
/// @return CRC disabled (0) or CRC enables (1), -1 for error
uint8_t TMAG5273::getCRCMode()
{
    uint8_t getCRC = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, getCRC) != ksfTkErrOk)
        return -1;

    return getBitFieldValue(getCRC, TMAG5273_CRC_MODE_BITS, TMAG5273_CRC_MODE_LSB);
}

/// @brief Returns the temp coefficient of the magnet
///     0X0 = 0% (No temperature compensation)
///     0X1 = 0.12%/deg C (NdBFe)
///     0X2 = Reserved
///     0X3 = 0.2%/deg C (Ceramic)
///     TMAG5273_REG_DEVICE_CONFIG_1 - bit 6-5
/// @return Temperature coefficient of the magnet
uint8_t TMAG5273::getMagTemp()
{
    uint8_t magTemp = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, magTemp) != ksfTkErrOk)
        return 1;

    // mast out mag bits
    return getBitFieldValue(magTemp, TMAG5273_MAG_TEMP_BITS, TMAG5273_MAG_TEMP_LSB);
}

/// @brief Returns the sampling of the sensor data to reduce
///  the noise effect (or to increase resolution)
///     0X0 = 1x average, 10.0-kSPS (3-axes) or 20-kSPS (1 axis)
///     0X1 = 2x average, 5.7-kSPS (3-axes) or 13.3-kSPS (1 axis)
///     0X2 = 4x average, 3.1-kSPS (3-axes) or 8.0-kSPS (1 axis)
///     0X3 = 8x average, 1.6-kSPS (3-axes) or 4.4-kSPS (1 axis)
///     0X4 = 16x average, 0.8-kSPS (3-axes) or 2.4-kSPS (1 axis)
///     0X5 =  32x average, 0.4-kSPS (3-axes) or 1.2-kSPS (1 axis)
///     TMAG5273_REG_DEVICE_CONFIG_1 - bit 4-2
/// @return Value of the sampling average of the sensor data
uint8_t TMAG5273::getConvAvg()
{
    uint8_t convAv = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, convAv) != ksfTkErrOk)
        return 1;

    return getBitFieldValue(convAv, TMAG5273_CONV_AVG_BITS, TMAG5273_CONV_AVG_LSB);
}

/// @brief Returns the I2C read mode
///     0X0 = Standard I2C 3-byte read command
///     0X1 = 1-byte I2C read command for 16bit sensor data and
///           conversion status
///     0X2 = 1-byte I2C read command for 8bit sensor MSB data and
///           conversion status
///     TMAG5273_REG_DEVICE_CONFIG_1 Bits 0-1
/// @return I2C read mode (0-2)
uint8_t TMAG5273::getReadMode()
{
    uint8_t readModeReg = 0;
    // Read the device config 1 register
    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_1, readModeReg) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(readModeReg, TMAG5273_I2C_READ_MODE_BITS, TMAG5273_I2C_READ_MODE_LSB);
}

/// @brief Returns the threshold for the interrupt function.
///     0X0 = Takes the 2's complement value of each x_THR_CONFIG
///      register to create a magnetic threshold of the corresponding axis
///     0X1 = Takes the 7LSB bits of the x_THR_CONFIG register to create
///      two opposite magnetic thresholds (one north, and another south)
///      of equal magnitude
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 7-5
/// @return Threshold for the interrupt function (0X0 - 0X1)
uint8_t TMAG5273::getIntThreshold()
{
    uint8_t interruptThreshold = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, interruptThreshold) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(interruptThreshold, TMAG5273_THR_HYST_BITS, TMAG5273_THR_HYST_LSB);
}

/// @brief Returns if the device is operating in low power
///  or low noise mode.
///     0X0 = Low active current mode
///     0X1 = Low noise mode
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 4
/// @return Low power (0) or low noise (1) mode
uint8_t TMAG5273::getLowPower()
{
    uint8_t lowPowerMode = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, lowPowerMode) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(lowPowerMode, TMAG5273_LOW_POWER_BITS, TMAG5273_LOW_POWER_LSB);
}

/// @brief Returns if the I2C glitch filter is on or off
///     0x0 = Glitch filter ON
///     0X0 = Glitch filter OFF
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 3
/// @return I2C filter ON (0) or OFF (1)
uint8_t TMAG5273::getGlitchFiler()
{
    uint8_t glitchMode = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, glitchMode) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(glitchMode, TMAG5273_GLITCH_FILTER_BITS, TMAG5273_GLITCH_FILTER_LSB);
}

/// @brief Returns the condition which initiates a single conversion
///  based off already configured registers. A running conversion
///  completes before executing a trigger. Redundant triggers are
///  ignored. TRIGGER_MODE is available only during the mode 'Stand-
///  by Mode' mentioned in OPERATING_MODE.
///     0X0 = Conversion Start at I2C Command Bits, DEFAULT
///     0X1 = Conversion starts through trigger signal at !INT pin
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 2
/// @return Condition that initiates conversion thru I2C or INT
uint8_t TMAG5273::getTriggerMode()
{
    uint8_t triggerMode = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, triggerMode) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(triggerMode, TMAG5273_TRIGGER_MODE_BITS, TMAG5273_TRIGGER_MODE_LSB);
}

/// @brief Returns the operating mode from one of the 4 listed below:
///     0X0 = Stand-by mode (starts new conversion at trigger event)
///     0X1 = Sleep mode
///     0X2 = Continuous measure mode
///     0X3 = Wake-up and sleep mode (W&S Mode)
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 1-0
/// @return Operating mode: stand-by, sleep, continuous, or wake-up and sleep
uint8_t TMAG5273::getOperatingMode()
{
    uint8_t opMode = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_CONFIG_2, opMode) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(opMode, TMAG5273_OPERATING_MODE_BITS, TMAG5273_OPERATING_MODE_LSB);
}

/// @brief Returns data acquisition from the following magnetic axis channels:
///     0X0 = All magnetic channels off, DEFAULT
///     0X1 = X Channel Enabled
///     0X2 = Y Channel Enabled
///     0X3 = X, Y Channel Enabled
///     0X4 = Z Channel Enabled
///     0X5 = Z, X Channel Enabled
///     0X6 = Y, Z Channel Enabled
///     0X7 = X, Y, Z Channel Enabled
///     0X8 = XYX Channel Enabled
///     0X9 = YXY Channel Enabled
///     0XA = YZY Channel Enabled
///     0XB = XZX Channel Enabled
///     TMAG5273_REG_SENSOR_CONFIG_1 - bit 7-4
/// @return Code for the magnetic channel axis being read
uint8_t TMAG5273::getMagneticChannel()
{
    uint8_t magChannel = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_1, magChannel) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(magChannel, TMAG5273_CHANNEL_MODE_BITS, TMAG5273_CHANNEL_MODE_LSB);
}

/// @brief Returns the time spent in low power mode between
///  conversions when OPERATING_MODE=11b.
///     0X0 = 1ms
///     0X1 = 5ms
///     0X2 = 10ms
///     0X3 = 15ms
///     0X4 = 20ms
///     0X5 = 30ms
///     0X6 = 50ms
///     0X7 = 100ms
///     0X8 = 500ms
///     0X9 = 1000ms
///     0XA = 2000ms
///     0XB = 5000ms
///     0XC = 20000ms
///     TMAG5273_REG_SENSOR_CONFIG_1 - bit 3-0
/// @return Code for the time spent in low power mode between conversions
uint8_t TMAG5273::getSleeptime()
{
    uint8_t sleepReg = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_1, sleepReg) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(sleepReg, TMAG5273_SLEEP_MODE_BITS, TMAG5273_SLEEP_MODE_LSB);
}

/// @brief Returns the direction of threshold check. This bit is
///  ignored when THR_HYST > 001b.
///     0X0 = sets interrupt for field above the threshold
///     0X1 = sets interrupt for field below the threshold
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 5
/// @return Direction of threshold check - 0 or 1
uint8_t TMAG5273::getMagDir()
{
    uint8_t magDirectionReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, magDirectionReg) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(magDirectionReg, TMAG5273_THRESHOLD_INT_BITS, TMAG5273_THRESHOLD_INT_LSB);
}

/// @brief Returns the axis for magnitude gain correction value
///  entered in MAG_GAIN_CONFIG register
///     0X0 = 1st channel is selected for gain adjustment
///     0X1 = 2nd channel is selected for gain adjustment
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 4
/// @return First (0) or Second (0) channel selected for gain adjustment
uint8_t TMAG5273::getMagnitudeChannelSelect()
{
    uint8_t magGainReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, magGainReg) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(magGainReg, TMAG5273_GAIN_ADJUST_BITS, TMAG5273_GAIN_ADJUST_LSB);
}

/// @brief This function returns an 8-bit gain value determined by a
///  primary to adjust a Hall axis gain. The particular axis is selected
///  based off the settings of MAG_GAIN_CH and ANGLE_EN register bits.
///  The binary 8-bit input is interpreted as a fractional value between
///  0 and 1 based off the formula 'user entered value in decimal/256.'
///  Gain value of 0 is interpreted by the device as 1.
///     TMAG5273_REG_MAG_GAIN_CONFIG
/// @return 8-bit gain value
uint8_t TMAG5273::getMagneticGain()
{
    uint8_t magneticGainReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_MAG_GAIN_CONFIG, magneticGainReg) != ksfTkErrOk)
        return 0;

    // okay, need to get the fractional bits into a floating point number
    // IEE754 format has the fraction as 23 bits - bit 22-0 - so shift left 15 and use those bits
    uint32_t iMagGain = magneticGainReg << 15;
    float fMagGain = 0;
    memcpy(&fMagGain, &iMagGain, sizeof(fMagGain));

    // So if everything is correct, fMagGain should be between 0 and 1
    // Multiply by resolution
    return (uint8_t)(fMagGain * 256.f);
}

/// @brief This function will return an 8-bit, 2's complement offset value
///  determined by a primary to adjust the first axis offset value. The
///  range of possible offset valid entrees can be +/-128. The offset value
///  is calculated by multiplying bit resolution with the entered value.
///     TMAG5273_REG_MAG_OFFSET_CONFIG_1
/// @return Magnetic offset value for the first axis
int8_t TMAG5273::getMagneticOffset1()
{
    uint8_t tmpRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_MAG_OFFSET_CONFIG_1, tmpRead) != ksfTkErrOk)
        return 0;
    int8_t magOffset1 = *(int8_t *)&tmpRead;

    // To calculate the offset register value from the desired offset in mT,
    // use the following equation from the datasheet:
    //   OFFSET_REG = (2^12 * OFFSET_mT) / 2*|RANGE_mT|
    // Where RANGE_mT is the full scale range of the selected axis
    // OFFSET_mT is the desired offset in mT - passed to this method
    //
    // factoring out the 2 in the denominator gives us:
    // OFFSET_REG = (2^11 * OFFSET_mT) / |RANGE_mT|
    //
    // Now to determine offset_mfT from OFFSET_REG, we rearrange to:
    //    OFFSET_mT = (OFFSET_REG * |RANGE_mT|) / 2^11

    uint8_t rangeValXY = getXYAxisRange();
    float range = rangeValXY == 0 ? 40.f : 80.f;

    return (int8_t)((magOffset1 * range) / 2048.f); // 2^11 = 2048
}

/// @brief This function will return an 8-bit, 2's complement offset value
///  determined by a primary to adjust the first axis offset value. The
///  range of possible offset valid entrees can be +/-128. The offset value
///  is calculated by multiplying bit resolution with the entered value.
///     TMAG5273_REG_MAG_OFFSET_CONFIG_2
/// @return Magnetic offset value for the second axis
int8_t TMAG5273::getMagneticOffset2()
{
    // deal with int/uint conversion
    uint8_t tempRead = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_MAG_OFFSET_CONFIG_2, tempRead) != ksfTkErrOk)
        return 0;

    int magOffset2 = *(int8_t *)&tempRead;

    // Determine whether the magnetic offset channel is Y or Z for the range selection
    uint8_t channelSelect = getAngleEn(); // 1, 2, or 3
    if (channelSelect < 1 || channelSelect > 3)
        return -1; // invalid channel selection

    // See which XY axis range is currently set

    float range;
    if (channelSelect == 1)
        range = getXYAxisRange() == 0 ? 40.f : 80.f;
    else
        range = getZAxisRange() == 0 ? 40.f : 80.f;

    // To calculate the offset register value from the desired offset in mT,
    // use the following equation from the datasheet:
    //   OFFSET_REG = (2^12 * OFFSET_mT) / 2*|RANGE_mT|
    // Where RANGE_mT is the full scale range of the selected axis
    // OFFSET_mT is the desired offset in mT - passed to this method
    //
    // factoring out the 2 in the denominator gives us:
    // OFFSET_REG = (2^11 * OFFSET_mT) / |RANGE_mT|
    //
    // Now to determine offset_mfT from OFFSET_REG, we rearrange to:
    //    OFFSET_mT = (OFFSET_REG * |RANGE_mT|) / 2^11

    // Use resolution equation in datasheet
    return (int8_t)((magOffset2 * range) / 2048.f); // 2^11 = 2048
}
// KDB ************************************************************************************************/
// STOP STOP STOP
// KDB ************************************************************************************************/

/// @brief Returns angle calculation, magnetic gain, and offset
///  corrections between two selected magnetic channels.
///     0X0 = No angle calculation, magnitude gain, and offset
///           correction enabled
///     0X1 = X 1st, Y 2nd
///     0X2 = Y 1st, Z 2nd
///     0X3 = X 1st, Z 2nd
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 3-2
/// @return Angle calculation and associated channel order
uint8_t TMAG5273::getAngleEn()
{
    uint8_t angleReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, angleReg) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(angleReg, TMAG5273_ANGLE_CALCULATION_BITS, TMAG5273_ANGLE_CALCULATION_LSB);
}

/// @brief Returns the X and Y axes magnetic range from the
///  two following options:
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 1
/// @return X and Y axes magnetic range (0 or 1)
uint8_t TMAG5273::getXYAxisRange()
{
    uint8_t xyRangeReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, xyRangeReg) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(xyRangeReg, TMAG5273_XY_RANGE_BITS, TMAG5273_XY_RANGE_LSB);
}

/// @brief Returns the Z axis magnetic range from the
///  two following options:
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 0
/// @return Z axis magnetic range from ±40mT or ±80mT
uint8_t TMAG5273::getZAxisRange()
{
    uint8_t zRangeReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_SENSOR_CONFIG_2, zRangeReg) != ksfTkErrOk)
        return 0;

    return getBitFieldValue(zRangeReg, TMAG5273_Z_RANGE_BITS, TMAG5273_Z_RANGE_LSB);
}

/// @brief Returns an 8-bit, 2's complement X axis threshold code for
///  limit check. The range of possible threshold entrees can be +/-128.
///  The threshold value in mT is calculated as (40(1+X_Y_RANGE)/128)*X_THR_CONFIG.
///  The value of X_THR_CONFIG is returned by this method
///  Default 0h means no threshold comparison.
///     TMAG5273_REG_X_THR_CONFIG - bits 7-0
/// @return Returns the X threshold code for limit check
int8_t TMAG5273::getXThreshold()
{

    uint8_t tempRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_X_THR_CONFIG, tempRead) != ksfTkErrOk)
        return 0;

    // convert uint8_t to int8_t and return
    return *(int8_t *)&tempRead;
}

/// @brief Returns an 8-bit, 2's complement Y axis threshold code for
///  limit check. The range of possible threshold entrees can be +/-128.
///  The threshold value in mT is calculated as (40(1+X_Y_RANGE)/128)*Y_THR_CONFIG.
///  The value of Y_THR_CONFIG is returned by this method
///  Default 0h means no threshold comparison.
///     TMAG5273_REG_Y_THR_CONFIG - bits 7-0
/// @return Returns the Y threshold code for limit check
int8_t TMAG5273::getYThreshold()
{

    uint8_t tempRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_Y_THR_CONFIG, tempRead) != ksfTkErrOk)
        return 0;

    // convert uint8_t to int8_t and return
    return *(int8_t *)&tempRead;
}

/// @brief Returns an 8-bit, 2's complement Z axis threshold code for
///  limit check. The range of possible threshold entrees can be +/-128.
///  The threshold value in mT is calculated as (40(1+Z_RANGE)/128)*Z_THR_CONFIG.
///  The value of Z_THR_CONFIG is returned by this method
///  Default 0h means no threshold comparison.
///     TMAG5273_REG_Z_THR_CONFIG - bits 7-0
/// @return Returns the Z threshold code for limit check
int8_t TMAG5273::getZThreshold()
{
    uint8_t tempRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_Z_THR_CONFIG, tempRead) != ksfTkErrOk)
        return 0;

    // convert uint8_t to int8_t and return
    return *(int8_t *)&tempRead;
}

/// @brief Returns the temperature threshold code entered by the user.
///  The valid temperature threshold values are -41C to 170C with the
///  threshold codes for -41C = 0x1A, and 170C = 0X34. Resolution is
///  8 degree C/LSB. Default 0x0 means no threshold comparison.
///     TMAG5273_REG_T_CONFIG - bits 7-1
/// @return Temperature threshold code entered by the user originally.
uint8_t TMAG5273::getTemperatureThreshold()
{

    uint8_t tempRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_T_CONFIG, tempRead) != ksfTkErrOk)
        return 0;

    // sift to start at bit 0, return the temperature threshold code value
    return tempRead >> 1;
}

/// @brief Returns the enable bit that determines the data
///  acquisition of the temperature channel.
///     0x0 = Temp Channel Disabled
///     0x1 = Temp Channel Enabled
///     TMAG5273_REG_T_CONFIG - bit 0
/// @return Enable bit that determines if temp channel is enabled or disabled
uint8_t TMAG5273::getTemperatureEN()
{
    uint8_t tempENreg = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_T_CONFIG, tempENreg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(tempENreg, TMAG5273_TEMPERATURE_BITS, TMAG5273_TEMPERATURE_LSB);
}

/// @brief Returns the enable interrupt response bit on conversion
///  complete.
///     0X0 = Interrupt is NOT asserted when the configured set of
///           conversions are complete
///     0X1 = Interrupt is asserted when the configured set of
///           conversions are complete
///     TMAG5273_REG_INT_CONFIG_1 - bit 7
/// @return Interrupt response bit for conversion complete.
uint8_t TMAG5273::getInterruptResult()
{
    uint8_t intRsltReg = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, intRsltReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(intRsltReg, TMAG5273_INTERRUPT_RESULT_BITS, TMAG5273_INTERRUPT_RESULT_LSB);
}

/// @brief Returns the bit that enables the interrupt response
///  on a predefined threshold cross.
///     0X0 = Interrupt is NOT asserted when a threshold is crossed
///     0X1 = Interrupt is asserted when a threshold is crossed
///     TMAG5273_REG_INT_CONFIG_1 - bit 6
/// @return Enable bit for if the interrupt is or is not asserted
uint8_t TMAG5273::getThresholdEn()
{
    uint8_t threshReg = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, threshReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(threshReg, TMAG5273_INTERRUPT_THRESHOLD_BITS, TMAG5273_INTERRUPT_THRESHOLD_LSB);
}

/// @brief Returns the !INT interrupt if it is latched or pulsed.
///     0X0 = !INT interrupt latched until clear by a primary
///           addressing the device
///     0X1 = !INT interrupt pulse for 10us
///     TMAG5273_REG_INT_CONFIG_1 - bit 5
/// @return Value if !INT interrupt is latched or pulsed
uint8_t TMAG5273::getIntPinState()
{
    uint8_t intStateReg = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, intStateReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(intStateReg, TMAG5273_INTERRUPT_PIN_STATE_BITS, TMAG5273_INTERRUPT_PIN_STATE_LSB);
}

/// @brief Returns the configuration for the interrupt mode select
///     0X0 = No interrupt
///     0X1 = Interrupt through !INT
///     0X2 = Interrupt through !INT except when I2C bus is busy
///     0X3 = Interrupt through SCL
///     0X4 = Interrupt through SCL except when I2C bus is busy
///     TMAG5273_REG_INT_CONFIG_1 - bit 4-2
/// @return Configuration for the interrupt mode select
uint8_t TMAG5273::getInterruptMode()
{
    uint8_t intModeReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, intModeReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(intModeReg, TMAG5273_INTERRUPT_MODE_BITS, TMAG5273_INTERRUPT_MODE_LSB);
}

/// @brief Returns the Mask !INT pin when !INT is connected to GND
///     0X0 = !INT pin is enabled
///     0X1 = !INT pin is disabled (for wake-up and trigger functions)
///     TMAG5273_REG_INT_CONFIG_1 - bit 0
/// @return !INT pin is enabled or disabled
uint8_t TMAG5273::getMaskInt()
{
    uint8_t maskIntReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_INT_CONFIG_1, maskIntReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(maskIntReg, TMAG5273_INTERRUPT_MASK_BITS, TMAG5273_INTERRUPT_MASK_LSB);
}

/// @brief Returns the rolling count of conversion data sets
///      TMAG5273_REG_CONV_STATUS - bit 7-5
/// @return Rolling count of conversion data sets
uint8_t TMAG5273::getSetCount()
{
    uint8_t convReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_CONV_STATUS, convReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(convReg, TMAG5273_CONV_STATUS_SET_COUNT_BITS, TMAG5273_CONV_STATUS_SET_COUNT_LSB);
}

/// @brief Returns if the device is powered up, or experienced power-
///  on-reset. Bit is cleared when the host writes back '1'.
///     0X0 = No POR
///     0X1 = POR occurred
///     TMAG5273_REG_CONV_STATUS - bit 4
/// @return Device is powered up or experienced POR.
uint8_t TMAG5273::getPOR()
{
    uint8_t convReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_CONV_STATUS, convReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(convReg, TMAG5273_CONV_STATUS_POR_BITS, TMAG5273_CONV_STATUS_POR_LSB);
}

/// @brief Returns if there was a detection of any internal
///  diagnostics fail which include VCC UV, internal memory CRC
///  error, !INT pin error and internal clock error. Ignore
///  this bit status if VCC < 2.3V.
///     0X0 = no diag fail
///     0X1 = diag fail detected
///     TMAG5273_REG_CONV_STATUS - bit 1
/// @return  Detection of any internal diagnostics failure
uint8_t TMAG5273::getDiagStatus()
{
    uint8_t convReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_CONV_STATUS, convReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(convReg, TMAG5273_CONV_STATUS_DIAG_STATUS_BITS, TMAG5273_CONV_STATUS_DIAG_STATUS_LSB);
}

/// @brief Returns if the conversion data buffer is ready
///  to be read.
///     0X0 = Conversion data not complete
///     0X1 = Conversion data complete
///     TMAG5273_REG_CONV_STATUS - bit 0
/// @return Conversion data buffer status
uint8_t TMAG5273::getResultStatus()
{
    uint8_t convReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_CONV_STATUS, convReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(convReg, TMAG5273_CONV_STATUS_RESULT_STATUS_BITS, TMAG5273_CONV_STATUS_RESULT_STATUS_LSB);
}

/// @brief Returns the I2C address of the device. There is a 7-bit
///  default factory address is loaded from OTP during first power
///  up. Change these bits to a new setting if a new I2C address is
///  required (at each power cycle these bits must be written again
///  to avoid going back to default factory address).
///     TMAG5273_REG_I2C_ADDRESS - bits 7-1
/// @return I2C address of the device
uint8_t TMAG5273::getI2CAddress()
{
    uint8_t addressReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_I2C_ADDRESS, addressReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(addressReg, TMAG5273_I2C_ADDRESS_BITS, TMAG5273_I2C_ADDRESS_LSB);
}

/// @brief Returns the device version indicator. The reset value of the
///  DEVICE_ID depends on the orderable part number
///     0X0 = Reserved
///     0X1 = ±40-mT and ±80-mT range
///     0X2 = ±133-mT and ±266-mT range
///     0X3 = Reserved
///     DEVICE_ID
/// @return Version number code of the device
uint8_t TMAG5273::getDeviceID()
{
    uint8_t deviceReg = 0;
    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_ID, deviceReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(deviceReg, TMAG5273_DEVICE_ID_BITS, TMAG5273_DEVICE_ID_LSB);
}

/// @brief Returns the 8-Bit Manufacturer ID. There are two
///  registers, LSB and MSB.
///     MANUFACTURER_ID_LSB
///     MANUFACTURER_ID_MSB
/// @return 16-Bit Manufacturer ID
uint16_t TMAG5273::getManufacturerID()
{
    uint8_t databuffer[2];
    size_t nRead;

    // Read both the LSB and MSB in one read. Results [LSB, MSB]
    if (_theI2CBus.readRegister(TMAG5273_REG_MANUFACTURER_ID_LSB, databuffer, 2, nRead) != ksfTkErrOk)
        return 0;

    // make a short word out of the two bytes
    return (databuffer[1] << 8) | (databuffer[0]);
}

/// @brief This function indicates the level that the device is
///  reading back from the !INT pin. The reset value of DEVICE_STATUS
/// depends on the status of the !INT pin at power-up.
/// @return Returns the following:
///     0X0 = !INT pin driven low
///     0X1 = !INT pin status high
uint8_t TMAG5273::getInterruptPinStatus()
{
    uint8_t deviceStatusReg = 0;

    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_STATUS, deviceStatusReg) != ksfTkErrOk)
        return 0;

    // clear out other bits, shift down to LSB and return
    return getBitFieldValue(deviceStatusReg, TMAG5273_DEVICE_STATUS_INTR_RB_BITS, TMAG5273_DEVICE_STATUS_INTR_RB_LSB);
}

/// @brief This function returns the device status register as its
///  raw hex value. This value can be taken and compared to the main
///  register as seen in the datasheet.
///  The errors include an oscillator error, INT pin error detected,
///  OTP CRC errors, or under voltage resistors.
///     TMAG5273_REG_DEVICE_STATUS
/// @return Device Status Register as a raw value.
uint8_t TMAG5273::getDeviceStatus()
{
    uint8_t deviceStatusReg = 0;
    // Check for under voltage, OTP CRC, Int Pin, and Oscillator errors
    if (_theI2CBus.readRegister(TMAG5273_REG_DEVICE_STATUS, deviceStatusReg) != ksfTkErrOk)
        return 0;

    return deviceStatusReg;
}

/// @brief This function will return the error code, if there is any
///  at the time when the function is called.
///  If any of the Error pins are raised to 1 then there is an error.
///  For more information on the specific error, checkout the
///  getDeviceStatus() function and compare to the datasheet.
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::getError()
{
    // Pull in the device status register to compare to the error codes
    uint8_t statusReg = getDeviceStatus();

    // Is the error bit set for any of the following:
    //   - Under voltage Error
    //   - OTP CRC Error
    //   - INT Pin Error
    //   - Oscillator Error

    if ((statusReg & (TMAG5273_DEVICE_STATUS_VCC_UV_ERROR_BITS | TMAG5273_DEVICE_STATUS_OTP_CRC_ERROR_BITS |
                      TMAG5273_DEVICE_STATUS_INT_ERROR_BITS | TMAG5273_OSCILLATOR_ERROR_BITS)) == 0)
        return 0; // no errors
    else
    {
        return -1; // error detected
    }
}

/************************************************************************************************/
/************************         MAGNETIC/TEMPERATURE FUNCTIONS         ************************/
/************************************************************************************************/

/// @brief Reads back the T-Channel data conversion results,
///  combining the MSB and LSB registers.
///     T_MSB_RESULT and T_LSB_RESULT
/// @return T-Channel data conversion results
float TMAG5273::getTemp()
{

    // Read the Temp data - returns the MSB and LSB in one read
    uint8_t dataBuffer[2];
    size_t nRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_T_MSB_RESULT, dataBuffer, 2, nRead) != ksfTkErrOk)
        return 0;

    // Combines the two in one register where the MSB is shifted to the correct location
    // convert to  int16_t (the data is in 2's complement format)
    int16_t tData = (dataBuffer[0] << 8) | dataBuffer[1];

    // Formula for correct output value - see equation 12 in the datasheet ...
    return TMAG5273_TSENSE_T0 + ((float)(tData - TMAG5273_TADC_T0) / (TMAG5273_TADC_RES));
}

///------------------------------------------------------------------------------------------------
/// @brief Reads back the X-Channel data conversion results, the
/// MSB 8-Bit and LSB 8-Bits. This reads from the following registers:
///     X_MSB_RESULT and X_LSB_RESULT
/// @return X-Channel data conversion results
float TMAG5273::getXData()
{
    // Read the X data - returns the MSB and LSB in one read
    uint8_t dataBuffer[2];
    size_t nRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_X_MSB_RESULT, dataBuffer, 2, nRead) != ksfTkErrOk)
        return 0;

    // Combines the two in one register where the MSB is shifted to the correct location
    // convert to  int16_t (the data is in 2's complement format)
    int16_t xData = (dataBuffer[0] << 8) | dataBuffer[1];

    // Reads to see if the range is set to 40mT or 80mT
    int32_t range = getXYAxisRange() == 0 ? 40 : 80;

    return calculateMagneticField(xData, range);
}

/// @brief Reads back the Y-Channel data conversion results, the
///  MSB 8-Bits and LSB 8-Bits. This reads from the following registers:
///     Y_MSB_RESULT and Y_LSB_RESULT
/// @return Y-Channel data conversion results
float TMAG5273::getYData()
{
    // Read the Y data - returns the MSB and LSB in one read
    uint8_t dataBuffer[2];
    size_t nRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_Y_MSB_RESULT, dataBuffer, 2, nRead) != ksfTkErrOk)
        return 0;

    // Combines the two in one register where the MSB is shifted to the correct location
    // convert to  int16_t (the data is in 2's complement format)
    int16_t yData = (dataBuffer[0] << 8) | dataBuffer[1];

    // Reads to see if the range is set to 40mT or 80mT
    int32_t range = getXYAxisRange() == 0 ? 40 : 80;

    return calculateMagneticField(yData, range);
}

/// @brief Reads back the Z-Channel data conversion results, the
///  MSB 8-Bits and LSB 8-Bits. This reads from the following registers:
///     Z_MSB_RESULT and Z_LSB_RESULT
/// @return Z-Channel data conversion results.
float TMAG5273::getZData()
{

    // Read the Z data - returns the MSB and LSB in one read
    uint8_t dataBuffer[2];
    size_t nRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_Z_MSB_RESULT, dataBuffer, 2, nRead) != ksfTkErrOk)
        return 0;

    // Combines the two in one register where the MSB is shifted to the correct location
    // convert to  int16_t (the data is in 2's complement format)
    int16_t zData = (dataBuffer[0] << 8) | dataBuffer[1];

    // Reads to see if the range is set to 40mT or 80mT
    int32_t range = getZAxisRange() == 0 ? 40 : 80;

    return calculateMagneticField(zData, range);
}

/// @brief Returns the angle measurement result in degree. The data
///  displayed from 0 to 360 degree in 13 LSB bits after combining the
///  MSB and LSB bits. The 4 LSB bits allocated for fraction of an angle
///  in the format (xxx/16).
///     TMAG5273_REG_ANGLE_RESULT_MSB
///     TMAG5273_REG_ANGLE_RESULT_LSB
/// @return Angle measurement result in degrees (float value)
float TMAG5273::getAngleResult()
{
    // Read the angle  data - returns the MSB and LSB in one read
    uint8_t dataBuffer[2];
    size_t nRead;
    if (_theI2CBus.readRegister(TMAG5273_REG_ANGLE_RESULT_MSB, dataBuffer, 2, nRead) != ksfTkErrOk)
        return 0;
    // Combines the two in one register where the MSB is shifted to the correct location
    // convert to  uint16_t (the data is in 2's complement format)
    uint16_t angleReg = (dataBuffer[0] << 8) | dataBuffer[1];

    // the data is formatted  in 13 bits, using 9.4 format (9 bits for integer, 4 bits for fraction (/16))
    //
    // fraction value - 4 LSB bits / 16
    float fractionValue = (float)(angleReg & 0xF) / 16.f;
    // integer value - shift off the bottom 4 bits, use. the next 9 bits
    float integerValue = (float)((angleReg >> 4) & 0x1FF);

    // return the combined value
    return integerValue + fractionValue;
}

/// @brief Returns the resultant vector magnitude (during angle
///  measurement) result. This value should be constant during 360
///  degree measurements.
/// @return Vector magnitude during angle measurement
float TMAG5273::getMagnitudeResult()
{
    // Creates a variable for the magnitude register
    uint8_t magReg = 0;

    // Reads in the magnitude result register
    if (_theI2CBus.readRegister(TMAG5273_REG_MAGNITUDE_RESULT, magReg) != ksfTkErrOk)
        return 0;

    return magReg;
}
