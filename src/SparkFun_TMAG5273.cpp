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
    uint16_t deviceID = readRegister();

	//make sure the device ID reported by the TMAG is correct
	//should always be 0x0D
	if (deviceID != DEVICE_ID_VALUE)
	{
		return false;
	}

/* ***** Figure this out *****
    // Check if Oscillator error is detected. Bit is clear when host writes back '1'
    if()
    {
        // 
    }

    // Check if !INT pin error is detected. Bit is clear when host writes back '1'
    if()
    {
        // 
    }

    // Check if OTP CRC error is detected. Bit is clear when host writes back '1'
    if()
    {
        // 
    }

    // Check if VCC undervoltage is detected. Occurs when VCC < 2.3V
    if()
    {

    }
*/

	return true; //returns true if all the checks pass
}


/* GET ADDRESS 
	This function returns the address of the device once
	called upon. This address can only be 0x22.
*/
uint8_t TMAG5273::getAddress()
{
	return _deviceAddress;
}


/* READ REGISTER
	This function reads the register bytes from the sensor when called upon.
	This reads 2 bytes of information from the 16-bit registers. 
*/
uint16_t TMAG5273::readRegister(uint8_t reg) // originally TMP117_Register reg
{
	_i2cPort->beginTransmission(_deviceAddress); // Originally cast (uint8_t)
	_i2cPort->write(reg);
	_i2cPort->endTransmission();					   // endTransmission but keep the connection active
	_i2cPort->requestFrom(_deviceAddress, (uint8_t)2); // Ask for 2 bytes, once done, bus is released by default

	uint8_t data[2] = {0};			// Declares an array of length 2 to be empty
	int16_t datac = 0;				// Declares the return variable to be 0
	if (_i2cPort->available() <= 2) // Won't read more than 2 bits
	{
		data[0] = _i2cPort->read();			// Reads the first set of bits (D15-D8)
		data[1] = _i2cPort->read();			// Reads the second set of bits (D7-D0)
		datac = ((data[0] << 8) | data[1]); // Swap the LSB and the MSB
	}
	return datac;
}

/* WRITE REGISTER
    Wire data to a TMAG5273 register
*/
void TMAG5273::writeRegister(uint8_t reg, uint16_t data) // originally TMP117_Register reg
{
	_i2cPort->beginTransmission(_deviceAddress); // Originally cast uint8_t when a register value again
	_i2cPort->write(reg);
	_i2cPort->write(highByte(data)); // Write MSB (D15-D8)
	_i2cPort->write(lowByte(data));  // Write LSB (D7-D0)
	_i2cPort->endTransmission();	 // Stop transmitting data
}


/************************************************************************************************/
/**************************        SET CONFIGURATION SETTINGS         ***************************/
/************************************************************************************************/

/* SET CRC I2C MODE
    This function sets the I2C CRC byte to be sent
    - 0X0 = CRC disabled
    - 0X1 = CRC enabled
    TMAG5273_DEVICE_CONFIG_1
*/
void TMAG5273::setCRC(uint8_t crc_mode)
{
    
}


/* SET MAGNET TEMPERATURE COEFFICIENT
    This function sets the temperature coefficient of the magnet
    - 0X0 = 0% (No temperature compensation)
    - 0X1 = 0.12%/deg C (NdBFe)
    - 0X2 = Reserved
    - 0X3 = 0.2%/deg C (Ceramic)
    TMAG5273_DEVICE_CONFIG_1
*/
void TMAG5273::setMagTemp(uint8_t magTemp_mode)
{

}


/* SET CONVERSION AVERAGE
    This function sets additional sampling of the sensor data
    to reduce the noise effect (or to increase resolution)
    - 0X0 = 1x average, 10.0-kSPS (3-axes) or 20-kSPS (1 axis)
    - 0X1 = 2x average, 5.7-kSPS (3-axes) or 13.3-kSPS (1 axis)
    - 0X2 = 4x average, 3.1-kSPS (3-axes) or 8.0-kSPS (1 axis)
    - 0X3 = 8x average, 1.6-kSPS (3-axes) or 4.4-kSPS (1 axis)
    - 0X4 = 16x average, 0.8-kSPS (3-axes) or 2.4-kSPS (1 axis)
    - 0X5 =  32x average, 0.4-kSPS (3-axes) or 1.2-kSPS (1 axis)
    TMAG5273_DEVICE_CONFIG_1
*/
void TMAG5273::setConvAvg(uint8_t avg_mode)
{

}


/* SET THRESHOLD SELECT
    This function sets the threshold for the interrupt function
    - 0X0 = Takes the 2's complement value of each x_THR_CONFIG
    register to create a magnetic threshold of the corresponding axis
    - 0X1 = Takes the 7LSB bits of the x_THR_CONFIG register to create
    two opposite magnetic thresholds (one north, and another south)
    of equal magnitude
    TMAG5273_DEVICE_CONFIG_2
*/
void TMAG5273::setIntThreshold(uint8_t threshold)
{

}


/* SET LOW POWER
    This function sets if the device is operatng in
    low opower or low noise mode
    - 0X0 = Low active current mode
    - 0X1 = Low noise mode
    TMAG5273_DEVICE_CONFIG_2
*/
void TMAG5273::setLowPower(uint8_t lp_ln_mode)
{

}

/* SET TRIGGER MODE
    This function sets a condition which initiates a single
    conversion based off already configured registers. A running 
    conversion completes before executing a trigger. Redundant
    triggers are ignored. TRIGGER_MODE is available only during 
    the mode 'Stand-by mode' mentioned in OPERATING_MODE.
    - 0X0 = Conversion Start at I2C Command Bits, DEFAULT
    - 0X1 = Conversion starts through trigger signal at !INT pin
    TMAG5273_DEVICE_CONFIG_2
*/
void TMAG5273::setTriggerMode(uint8_t trig_mode)
{

}


/* SET OPERATING MODE
    This function sets the operating mode from one
    of the 4 listed below: 
    - 0X0 = Stand-by mode (starts new conversion at trigger event)
    - 0X1 = Sleep mode
    - 0X2 = Continuous measure mode
    - 0X3 = Wake-up and sleep mode (W&S Mode)
    TMAG5273_DEVICE_CONFIG_2
*/
void TMAG5273::setOperatingMode(uint8_t op_mode)
{

}


/* SET MAGNETIC CHANNEL
    This function sets data acquisition from the following 
    magnetic axis channels: 
    - 0X0 = All magnetic channels off, DEFAULT
    - 0X1 = X Channel Enabled
    - 0X2 = Y Channel Enabled
    - 0X3 = X, Y Channel Enabled
    - 0X4 = Z Channel Enabled
    - 0X5 = Z, X Channel Enabled
    - 0X6 = Y, Z Channel Enabled
    - 0X7 = X, Y, Z Channel Enabled
    - 0X8 = XYX Channel Enabled
    - 0X9 = YXY Channel Enabled
    - 0XA = YZY Channel Enabled
    - 0XB = XZX Channel Enabled 
    TMAG5273_SENSOR_CONFIG_1
*/
void TMAG5273::setMagChannel(uint16_t channel_mode)
{
    
}


/* SET SLEEPTIME 
    This function sets the time spent in low power
    mode between conversions when OPERATING_MODE=11b
    - 0X0 = 1ms
    - 0X1 = 5ms
    - 0X2 = 10ms
    - 0X3 = 15ms
    - 0X4 = 20ms
    - 0X5 = 30ms
    - 0X6 = 50ms
    - 0X7 = 100ms
    - 0X8 = 500ms
    - 0X9 = 1000ms
    - 0XA = 2000ms
    - 0XB = 5000ms
    - 0XC = 20000ms
    TMAG5273_SENSOR_CONFIG_1
*/
void TMAG5273::setSleeptime(uint16_t sleepTime)
{

}


/* SET DIRECTION OF THRESHOLD CHECK
    This function sets the direction of threshold check. This
    bit is ignored when THR_HYST > 001b
    - 0X0 = sets interrupt for field above the threshold
    - 0X1 = sets interrupt for field below the threshold
    TMAG5273_SENSOR_CONFIG_2
*/
void TMAG5273::setMagDir(uint8_t thresh_dir)
{

}


/* SET MAGNITUDE GAIN
    This function sets the axis for magnitude gain correction
    value entered in MAG_GAIN_CONFIG register
    - 0X0 = 1st channel is selected for gain adjustment
    - 0X1 = 2nd channel is selected for gain adjustment
    TMAG5273_SENSOR_CONFIG_2
*/
void TMAG5273::setMagGain(uint8_t gain_adjust)
{

}


/* SET ANGLE CALCULATION
    This function sets angle calculation, magnetic gain, and 
    offset corrections between two selected magnetic channels
    - 0X0 = No angle calculation, magnitude gain, and offset 
            correction enabled
    - 0X1 = X 1st, Y 2nd
    - 0X2 = Y 1st, Z 2nd
    - 0X3 = X 1st, Z 2nd
    TMAG5273_SENSOR_CONFIG_2
*/
void TMAG5273::setAngleEn(uint8_t angle_en)
{

}


/* SET XY AXIS RANGE
    This function sets the X and Y axes magnetic range from 
    2 different options: 
    - 0X0 = ±40mT, DEFAULT
    - 0X1 = ±80mT
    TMAG5273_SENSOR_CONFIG_2
*/
void TMAG5273::setXYAxisRange(uint8_t XY_axis_range)
{

}


/* SET Z AXIS RANGE SELECT
    This function sets the Z axis magnetic range from 
    2 different options: 
    - 0X0 = ±40mT, DEFAULT
    - 0X1 = ±80mT
    TMAG5273_SENSOR_CONFIG_2
*/
void TMAG5273::setZAxisRange(uint8_t Z_axis_range)
{

}


/* SET X THRESHOLD
    This function sets an 8-bit, 2's complement X axis threshold
    code for limit check. The range of possible threshold entrees
    can be +/-128. The threshold value in mT is caluclated as 
    (40(1+X_Y_RANGE)/128)*X_THR_CONFIG. Defualt 0h means no 
    threshold comparison
    TMAG5273_X_THR_CONFIG
*/
void TMAG5273::setXThresh(uint8_t X_thresh)
{

}


/* SET THRESHOLD
    This function sets an 8-bit, 2's complement Y axis threshold
    code for limit check. The range of possible threshold entrees
    can be +/-128. The threshold value in mT is caluclated as 
    (40(1+X_Y_RANGE)/128)*Y_THR_CONFIG. Defualt 0h means no 
    threshold comparison
    TMAG5273_Y_THR_CONFIG
*/
void TMAG5273::setYThresh(uint8_t Y_thresh)
{
    
}


/* SET Z THRESHOLD
    This function sets an 8-bit, 2's complement Z axis threshold
    code for limit check. The range of possible threshold entrees
    can be +/-128. The threshold value in mT is caluclated as 
    (40(1+Z_RANGE)/128)*Z_THR_CONFIG. Defualt 0h means no 
    threshold comparison
    TMAG5273_Z_THR_CONFIG
*/
void TMAG5273::setZThresh(uint8_t Z_thresh)
{
    
}

/* SET TEMPERATURE THRESHOLD
    This function sets the temperature threshold code entered by the 
    user. The valid temperature threshold ranges are -41C to 
    170C with the threshold codes for -41C = 0X1A, and 170C = 0X34.
    Resultion is 8 degree C/LSB. Default 0X0 means no threshold
    comparison.
    TMAG5273_T_CONFIG
*/
void TMAG5273::setTempThresh(uint8_t temp_thresh)
{

}


/* SET INTERRUPT RESULT
    This function sets the enable interrupt response bit on 
    conversion complete. 
    - 0X0 = Interrupt is NOT asserted when the configured set of
            conversions are complete
    - 0X1 = Interrupt is asserted when the configured set of 
            conversions are complete
    TMAG5273_INT_CONFIG_1
*/
void TMAG5273::setIntRslt(uint8_t int_en)
{

}


/* SET ENABLE INTERRUPT RESPONSE THRESHOLD
    This function configures the bit that enables interrupt resoonse
    on a predefined threshold cross.
    - 0X0 = Interrupt is NOT asserted when a threshold is crossed
    - 0X1 = Interrupt is asserted when a threshold is crossed
    TMAG5273_INT_CONFIG_1
*/
void TMAG5273::setThreshEn(uint8_t en_int_response)
{

}


/* SET INTERRUPT STATE
    This function enables the !INT interrupt if it is latched
    or pulsed. 
    - 0X0 = !INT interrupt latched until clear by a primary
            addressing the device
    - 0X1 = !INT interrupt pulse for 10us
    TMAG5273_INT_CONFIG_1
*/
void TMAG5273::setIntState(uint8_t int_state)
{

}


/* SET INTERRUPT MODE
    This function configures the interrupt mode select.
    - 0X0 = No interrupt
    - 0X1 = Interrupt through !INT
    - 0X2 = Interrupt through !INT except when I2C bus is busy
    - 0X3 = Interrupt through SCL
    - 0X4 = Interrupt through SCL except when I2C bus is busy
    TMAG5273_INT_CONFIG_1
*/
void TMAG5273::setIntMode(uint8_t config_mode)
{

}


/* SET MASK INT BIT
    This function configures the Mask !INT pin when !INT
    is connected to GND
    - 0X0 = !INT pin is enabled
    - 0X1 = !INT pin is disabled (for wake-up and trigger functions)
    TMAG5273_INT_CONFIG_1
*/
void TMAG5273::setMaskInt(uint8_t int_en)
{

}


/************************************************************************************************/
/**************************        GET CONFIGURATION SETTINGS         ***************************/
/************************************************************************************************/


/* GET MAGNET TEMPERATURE COEFFICIENT
    This function returns the temperature coefficient of the magnet
    - 0X0 = 0% (No temperature compensation)
    - 0X1 = 0.12%/deg C (NdBFe)
    - 0X2 = Reserved
    - 0X3 = 0.2%/deg C (Ceramic)
    TMAG5273_DEVICE_CONFIG_1 - bit 6-5
*/
uint16_t TMAG5273::getMagTemp()
{
    uint16_t magTemp = 0;
    magTemp = readRegister(TMAG5273_DEVICE_CONFIG_1);

    uint8_t tempReg5 = bitRead(magTemp, 5); // Fills with bit 5
    uint8_t tempReg6 = bitRead(magTemp, 6); // Fills with bit 6

    if((tempReg5 == 0) && (tempReg6 == 0)) // 0b00
    {
        return 0; // 0%
    }
    else if((tempReg5 == 1) && (tempReg6 == 0)) // 0b01
    {
        return 1; // 0.12%/deg C
    }
    else if((tempReg5 == 0) && (tempReg6 == 1)) // 0b10
    {
        return 2; // Reserved
    }
    else if((tempReg5 == 1) && (tempReg == 1)) // 0b11
    {
        return 3; // 0.2%/deg C
    }
    else
    {
        return 1; // default
    }
}


/* GET CONVERSION AVERAGE
    This function returns the sampling of the sensor data
    to reduce the noise effect (or to increase resolution)
    - 0X0 = 1x average, 10.0-kSPS (3-axes) or 20-kSPS (1 axis)
    - 0X1 = 2x average, 5.7-kSPS (3-axes) or 13.3-kSPS (1 axis)
    - 0X2 = 4x average, 3.1-kSPS (3-axes) or 8.0-kSPS (1 axis)
    - 0X3 = 8x average, 1.6-kSPS (3-axes) or 4.4-kSPS (1 axis)
    - 0X4 = 16x average, 0.8-kSPS (3-axes) or 2.4-kSPS (1 axis)
    - 0X5 =  32x average, 0.4-kSPS (3-axes) or 1.2-kSPS (1 axis)
    TMAG5273_DEVICE_CONFIG_1 - bit 4-2
*/
uint16_t TMAG5273::getConvAvg()
{
    uint16_t convAv = 0;
    convAv = readRegister(TMAG5273_DEVICE_CONFIG_1);

    uint8_t convBit2 = bitRead(convAv, 2);
    uint8_t convBit3 = bitRead(convAv, 3);
    uint8_t convBit4 = bitRead(convAv, 4);

    if((convBit2 == 0) && (convBit3 == 0) && (convBit4 == 0)) // 0b000
    {
        return 0; // 1x
    }
    else if((convBit2 == 1) && (convBit3 == 0) && (convBit4 == 0)) // 0b001
    {
        return 1; // 2x
    }
    else if((convBit2 == 0) && (convBit3 == 1) && (convBit4 == 0)) // 0b010
    {
        return 2; // 4x
    }
    else if((convBit2 == 1) && (convBit3 == 1) && (convBit4 == 0)) // 0b011
    {
        return 3; // 8x
    }
    else if((convBit2 == 1) && (convBit3 == 0) && (convBit4 == 0)) // 0b100
    {
        return 4; // 16x
    }
    else if((convBit2 == 1) && (convBit3 == 0) && (convBit4 == 1)) // 0b001
    {
        return 5; // 32x
    }
    else
    {
        return 1; // default
    }
}


/* GET THRESHOLD SELECT
    This function returns the threshold for the interrupt function
    - 0X0 = Takes the 2's complement value of each x_THR_CONFIG
    register to create a magnetic threshold of the corresponding axis
    - 0X1 = Takes the 7LSB bits of the x_THR_CONFIG register to create
    two opposite magnetic thresholds (one north, and another south)
    of equal magnitude
    TMAG5273_DEVICE_CONFIG_2 - bit 7-5
*/
uint16_t TMAG5273::getIntThreshold()
{
    uint16_t intThresh = 0;
    intThresh = readRegister(TMAG5273_DEVICE_CONFIG_2);

    uint8_t intThresh5 = bitRead(intThresh, 5);
    uint8_t intThresh6 = bitRead(intThresh, 6);
    uint8_t intThresh7 = bitRead(intThresh, 7);

    if((intThresh5 == 0) && (intThresh6 == 0) && (intThresh7 == 0)) // 0b000
    {
        return 0; // 2's complement
    }
    else if((intThresh5 == 1) && (intThresh6 == 0) && (intThresh7 == 0)) // 0b001
    {
        return 1; // 7LSB for two opposite mag thresholds
    }
    else
    {
        return 0; // default
    }
}


/* Get LOW POWER
    This function selects if the device is operatng in
    low opower or low noise mode
    - 0X0 = Low active current mode
    - 0X1 = Low noise mode
    TMAG5273_DEVICE_CONFIG_2 - bit 4
*/
uint16_t TMAG5273::getLowPower()
{
    uint16_t lpMode = 0;
    lpMode = readRegister(TMAG5273_DEVICE_CONFIG_2, 4);

    uint8_t lpModeBit = bitRead(lpMode, 4);

    return lpModeBit;
}


/* GET TRIGGER MODE
    This function returns the condition which initiates a single
    conversion based off already configured registers. A running 
    conversion completes before executing a trigger. Redundant
    triggers are ignored. TRIGGER_MODE is available only during 
    the mode 'Stand-by mode' mentioned in OPERATING_MODE.
    - 0X0 = Conversion Start at I2C Command Bits, DEFAULT
    - 0X1 = Conversion starts through trigger signal at !INT pin
    TMAG5273_DEVICE_CONFIG_2 - bit 2
*/
uint16_t TMAG5273::getTriggerMode()
{
    uint16_t trigMode = 0;
    trigMode = readRegister(TMAG5273_DEVICE_CONFIG_2);

    uint8_t trigModeBit = bitRead(trigMode, 2);

    return trigModeBit;
}


/* GET OPERATING MODE
    This function returns the operating mode from one
    of the 4 listed below: 
    - 0X0 = Stand-by mode (starts new conversion at trigger event)
    - 0X1 = Sleep mode
    - 0X2 = Continuous measure mode
    - 0X3 = Wake-up and sleep mode (W&S Mode)
    TMAG5273_DEVICE_CONFIG_2 - bit 1-0
*/
uint16_t TMAG5273::getOperatingMode()
{
    uint16_t opMode = 0;
    opMode = readRegister(TMAG5273_DEVICE_CONFIG_2);

    uint8_t opMode0 = bitRead(opMode, 0);
    uint8_t opMode1 = bitRead(opMode, 1);

    if((opMode0 == 0) && (opMode1 == 0)) // 0b00
    {
        return 0; // Stand-by mode
    }
    else if((opMode0 == 1) && (opMode1 == 0)) // 0b01
    {
        return 1; // Sleep mode
    }
    else if((opMode0 == 0) && (opMode1 == 1)) // 0b10
    {
        return 2; // Continuous measure mode
    }
    else if((opMode0 == 1) && (opMode1 == 1)) // 0b11
    {
        return 3; // Wake-up and sleep mode (W&S Mode)
    }
}


/* GET MAGNETIC CHANNEL
    This function returns data acquisition from the following 
    magnetic axis channels: 
    - 0X0 = All magnetic channels off, DEFAULT
    - 0X1 = X Channel Enabled
    - 0X2 = Y Channel Enabled
    - 0X3 = X, Y Channel Enabled
    - 0X4 = Z Channel Enabled
    - 0X5 = Z, X Channel Enabled
    - 0X6 = Y, Z Channel Enabled
    - 0X7 = X, Y, Z Channel Enabled
    - 0X8 = XYX Channel Enabled
    - 0X9 = YXY Channel Enabled
    - 0XA = YZY Channel Enabled
    - 0XB = XZX Channel Enabled
    TMAG5273_SENSOR_CONFIG_1 - bit 7-4
*/
uint16_t TMAG5273::getMagChannel()
{
    uint16_t magChannel = 0;
    magChannel = readRegister(TMAG5273_SENSOR_CONFIG_1);

    uint8_t magMode4 = bitRead(magChannel, 4);
    uint8_t magMode5 = bitRead(magChannel, 5);
    uint8_t magMode6 = bitRead(magChannel, 6);
    uint8_t magMode7 = bitRead(magChannel, 7); 

    if((magMode4 == 0) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 0)) // 0b0000
    {
        return 0; // All mag channels off
    }
    else if((magMode4 == 1) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 0)) // 0b0001
    {
        return 1; // X channel enabled
    }
    else if((magMode4 == 0) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 0)) // 0b0010
    {
        return 2; // Y channel enabled
    }
    else if((magMode4 == 1) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 0)) // 0b0011
    {
        return 3; // X, Y channel enabled
    }
    else if((magMode4 == 0) && (magMode5 == 0) && (magMode6 == 1) && (magMode7 == 0)) // 0b0100
    {
        return 4; // Z channel enabled
    }
    else if((magMode4 == 1) && (magMode5 == 0) && (magMode6 == 1) && (magMode7 == 0)) // 0b0101
    {
        return 5; // Z, X channel enabled
    }
    else if((magMode4 == 0) && (magMode5 == 1) && (magMode6 == 1) && (magMode7 == 0)) // 0b0110
    {
        return 6; // Y, Z channel enabled
    }
    else if((magMode4 == 1) && (magMode5 == 1) && (magMode6 == 1) && (magMode7 == 0)) // 0b0111
    {
        return 7; // X, Y, Z channel enabled
    }
    else if((magMode4 == 0) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 1)) // 0b1000
    {
        return 8; // XYX channel enabled
    }
    else if((magMode4 == 1) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 1)) // 0b1001
    {
        return 9; // YXY channel enabled
    }
    else if((magMode4 == 0) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 1)) // 0b1010
    {
        return 10; // YZY channel enabled
    }
    else if((magMode4 == 1) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 1)) // 0b1011
    {
        return 11; // XZX channel enabled
    }
    else
    {
        return 0; // DEFAULT
    }
}


/* GET SLEEPTIME 
    This function returns the time spent in low power
    mode between conversions when OPERATING_MODE=11b
    - 0X0 = 1ms
    - 0X1 = 5ms
    - 0X2 = 10ms
    - 0X3 = 15ms
    - 0X4 = 20ms
    - 0X5 = 30ms
    - 0X6 = 50ms
    - 0X7 = 100ms
    - 0X8 = 500ms
    - 0X9 = 1000ms
    - 0XA = 2000ms
    - 0XB = 5000ms
    - 0XC = 20000ms
    TMAG5273_SENSOR_CONFIG_1 - bit 3-0
*/
uint16_t TMAG5273::getSleeptime()
{
    uint16_t sleepReg  = 0;
    sleepReg = readRegister(TMAG5273_SENSOR_CONFIG_1);

    uint8_t sleep0 = bitRead(sleepReg, 0);
    uint8_t sleep1 = bitRead(sleepReg, 1);
    uint8_t sleep2 = bitRead(sleepReg, 2);
    uint8_t sleep3 = bitRead(sleepReg, 3);

    if((sleep0 == 0) && (sleep1 == 0) && (sleep2 == 0) && (sleep3 == 0)) // 0b0000
    {
        return 0; // 1ms
    }
    else if((sleep0 == 1) && (sleep1 == 0) && (sleep2 == 0) && (sleep3 == 0)) // 0b0001
    {
        return 1; // 5ms
    }
    else if((sleep0 == 0) && (sleep1 == 1) && (sleep2 == 0) && (sleep3 == 0)) // 0b0010
    {
        return 2; // 10ms
    }
    else if((sleep0 == 1) && (sleep1 == 1) && (sleep2 == 0) && (sleep3 == 0)) // 0b0011
    {
        return 3; // 15ms
    }
    else if((sleep0 == 0) && (sleep1 == 0) && (sleep2 == 1) && (sleep3 == 0)) // 0b0100
    {
        return 4; // 20ms
    }
    else if((sleep0 == 1) && (sleep1 == 0) && (sleep2 == 1) && (sleep3 == 0)) // 0b0101
    {
        return 5; // 30ms
    }
    else if((sleep0 == 0) && (sleep1 == 1) && (sleep2 == 1) && (sleep3 == 0)) // 0b0110
    {
        return 6; // 50ms
    }
    else if((sleep0 == 1) && (sleep1 == 1) && (sleep2 == 1) && (sleep3 == 0)) // 0b0111
    {
        return 7; // 100ms
    }
    else if((sleep0 == 0) && (sleep1 == 0) && (sleep2 == 0) && (sleep3 == 1)) // 0b1000
    {
        return 8; // 500ms
    }
    else if((sleep0 == 1) && (sleep1 == 0) && (sleep2 == 0) && (sleep3 == 1)) // 0b1001
    {
        return 9; // 1000ms
    }
    else if((sleep0 == 0) && (sleep1 == 1) && (sleep2 == 0) && (sleep3 == 1)) // 0b1010
    {
        return 10; // 2000ms
    }
    else if((sleep0 == 1) && (sleep1 == 1) && (sleep2 == 0) && (sleep3 == 1)) // 0b1011
    {
        return 11; // 5000ms
    }
    else if((sleep0 == 0) && (sleep1 == 0) && (sleep2 == 1) && (sleep3 == 1)) // 0b1011
    {
        return 11; // 20000ms
    }
    else
    {
        return 0; // DEFAULT
    }
}


/* GET DIRECTION OF THRESHOLD CHECK
    This function returns the direction of threshold check. This
    bit is ignored when THR_HYST > 001b
    - 0X0 = sets interrupt for field above the threshold
    - 0X1 = sets interrupt for field below the threshold
    TMAG5273_SENSOR_CONFIG_2 - bit 5
*/
uint16_t TMAG5273::setMagDir()
{
    uint16_t magDirReg = 0;
    magDirReg = readRegister(TMAG5273_SENSOR_CONFIG_2);

    uint8_t magDir5 = bitRead(magDirReg, 5);

    return magDir5; 
}


/* GET MAGNITUDE GAIN
    This function returns the axis for magnitude gain correction
    value entered in MAG_GAIN_CONFIG register
    - 0X0 = 1st channel is selected for gain adjustment
    - 0X1 = 2nd channel is selected for gain adjustment
    TMAG5273_SENSOR_CONFIG_2 - bit 4
*/
uint16_t TMAG5273::getMagGain()
{
    uint16_t magGainReg = 0;
    magGainReg = readRegister(TMAG5273_SENSOR_CONFIG_2);

    uint8_t magGain4 = bitRead(magGainReg, 4);

    return magGain4;
}


/* GET ANGLE CALCULATION
    This function returns angle calculation, magnetic gain, and 
    offset corrections between two selected magnetic channels
    - 0X0 = No angle calculation, magnitude gain, and offset 
            correction enabled
    - 0X1 = X 1st, Y 2nd
    - 0X2 = Y 1st, Z 2nd
    - 0X3 = X 1st, Z 2nd
    TMAG5273_SENSOR_CONFIG_2 - bit 3-2
*/
uint16_t TMAG5273::getAngleEn()
{
    uint16_t angleReg = 0;
    angleReg = readRegister(TMAG5273_SENSOR_CONFIG_2);

    uint8_t angleDir2 = bitRead(angleReg, 2);
    uint8_t angleDir3 = bitRead(angleReg, 3);

    if((angleDir2 == 0) && (angleDir3 == 0)) // 0b00
    {
        return 0; // NO angle calculation
    }
    else if((angleDir2 == 1) && (angleDir3 == 0)) // 0b01
    {
        return 1; // X 1st, Y 2nd
    }
    else if((angleDir2 == 0) && (angleDir3 == 1)) // 0b10
    {
        return 2; // Y 1st, Z 2nd
    }
    else if((angleDir2 == 1) && (angleDir3 == 1)) // 0b11
    {
        return 3; // X 1st, Z 2nd
    }
}


/* GET XY AXIS RANGE
    This function return the X and Y axes magnetic range from 
    2 different options: 
    - 0X0 = ±40mT, DEFAULT
    - 0X1 = ±80mT
    TMAG5273_SENSOR_CONFIG_2 - bit 1
*/
uint16_t TMAG5273::getXYAxisRange()
{
    uint16_t XYrangeReg = 0;
    XYrangeReg = readRegister(TMAG5273_SENSOR_CONFIG_2);
    
    uint8_t axisRange = bitRead(XYrangeReg, 1);

    return axisRange;
}


/* GET Z AXIS RANGE
    This function returns the Z axis magnetic range from 
    2 different options: 
    - 0X0 = ±40mT, DEFAULT
    - 0X1 = ±80mT
    TMAG5273_SENSOR_CONFIG_2 - bit 0
*/
uint16_t TMAG5273::getZAxisRange()
{
    uint16_t ZrangeReg = 0;
    ZrangeReg = readRegister(TMAG5273_SENSOR_CONFIG_2);
    
    uint8_t ZaxisRange = bitRead(ZrangeReg, 0);

    return ZaxisRange;
}


/* GET X THRESHOLD
    This function returns an 8-bit, 2's complement X axis threshold
    code for limit check. The range of possible threshold entrees
    can be +/-128. The threshold value in mT is caluclated as 
    (40(1+X_Y_RANGE)/128)*X_THR_CONFIG. Defualt 0h means no 
    threshold comparison
    TMAG5273_X_THR_CONFIG - bits 7-0
*/
uint16_t TMAG5273::getXThresh()
{
    uint16_t xThresh = 0;
    xThresh = readRegister(TMAG5273_X_THR_CONFIG);

    uint16_t X_Y_RANGE = getXYAxisRange();
    uint16_t threshVal = (40(1+X_Y_RANGE)/128)*xThresh;

    return threshVal;
}


/* GET THRESHOLD
    This function returns an 8-bit, 2's complement Y axis threshold
    code for limit check. The range of possible threshold entrees
    can be +/-128. The threshold value in mT is caluclated as 
    (40(1+X_Y_RANGE)/128)*Y_THR_CONFIG. Defualt 0h means no 
    threshold comparison
    TMAG5273_Y_THR_CONFIG - bits 7-0
*/
uint16_t TMAG5273::getYThresh()
{
    uint16_t yThresh = 0;
    yThresh = readRegister(TMAG5273_Y_THR_CONFIG);

    uint16_t X_Y_RANGE = getXYAxisRange();
    uint16_t threshVal = (40(1+X_Y_RANGE)/128)*yThresh;

    return threshVal;
}



/* GET Z THRESHOLD
    This function returns an 8-bit, 2's complement Z axis threshold
    code for limit check. The range of possible threshold entrees
    can be +/-128. The threshold value in mT is caluclated as 
    (40(1+Z_RANGE)/128)*Z_THR_CONFIG. Defualt 0h means no 
    threshold comparison
    TMAG5273_Z_THR_CONFIG - bits 7-0
*/
uint16_t TMAG5273::getZThresh()
{
    uint16_t zThresh = 0;
    zThresh = readRegister(TMAG5273_Z_THR_CONFIG);

    uint16_t Z_RANGE = getZAxisRange();
    uint16_t threshVal = (40(1+Z_RANGE)/128)*zThresh;

    return threshVal;
}


/* GET TEMPERATURE THRESHOLD
    This function returns the temperature threshold code entered by the 
    user. The valid temperature threshold ranges are -41C to 
    170C with the threshold codes for -41C = 0X1A, and 170C = 0X34.
    Resultion is 8 degree C/LSB. Default 0X0 means no threshold
    comparison.
    TMAG5273_T_CONFIG - bits 7-1
*/
uint16_t TMAG5273::getTempThresh()
{
    uint16_t tempThreshReg = 0;
    tempThreshReg = readRegister(TMAG5273_T_CONFIG);

    uint8_t tempThresh = 0b0;

    //**CONCATENATE BITS 7:1 INTO EMPTY REGISTER
}

/* GET TEMPERATURE THRESHOLD ENABLE
    This function returns the enable bit that determines the data
    acquisition of the temperature channel.
    - 0x0 = Temp Channel Disabled
    - 0x1 = Temp Channel Enabled
    TMAG5273_T_CONFIG - bit 0
*/
uint16_t TMAG5273::getTempThreshEN()
{
    uint16_t tempENreg = 0;
    tempENreg = readRegister(TMAG5273_T_CONFIG);

    uint8_t tempEN = bitRead(tempENreg, 0);

    return tempEN;
}


/* GET INTERRUPT RESULT
    This function returns the enable interrupt response bit on 
    conversion complete. 
    - 0X0 = Interrupt is NOT asserted when the configured set of
            conversions are complete
    - 0X1 = Interrupt is asserted when the configured set of 
            conversions are complete
    TMAG5273_INT_CONFIG_1 - bit 7
*/
uint16_t TMAG5273::getIntRslt()
{
    uint16_t intRsltReg = 0;
    intRsltReg = readRegister(TMAG5273_INT_CONFIG_1);

    uint8_t intRslt = bitRead(intRsltReg, 7);

    return intRslt;
}


/* GET ENABLE INTERRUPT RESPONSE THRESHOLD
    This function configures the bit that enables interrupt resoonse
    on a predefined threshold cross.
    - 0X0 = Interrupt is NOT asserted when a threshold is crossed
    - 0X1 = Interrupt is asserted when a threshold is crossed
    TMAG5273_INT_CONFIG_1 - bit 6
*/
uint16_t TMAG5273::getThreshEn()
{
    uint16_t threshReg = 0;
    threshReg = readRegister(TMAG5273_INT_CONFIG_1);

    uint8_t threshEnRslt = bitRead(threshReg, 6);

    return threshEnRslt;
}


/* GET INTERRUPT STATE
    This function enables the !INT interrupt if it is latched
    or pulsed. 
    - 0X0 = !INT interrupt latched until clear by a primary
            addressing the device
    - 0X1 = !INT interrupt pulse for 10us
    TMAG5273_INT_CONFIG_1 - bit 5
*/
uint16_t TMAG5273::getIntState()
{
    uint16_t intStateReg = 0;
    intStateReg = readRegister(TMAG5273_INT_CONFIG_1);

    uint8_t intStateRslt = bitRead(intStateReg, 5);

    return intStateRslt;
}


/* GET INTERRUPT MODE
    This function configures the interrupt mode select.
    - 0X0 = No interrupt
    - 0X1 = Interrupt through !INT
    - 0X2 = Interrupt through !INT except when I2C bus is busy
    - 0X3 = Interrupt through SCL
    - 0X4 = Interrupt through SCL except when I2C bus is busy
    TMAG5273_INT_CONFIG_1 - bit 4-2
*/
uint16_t TMAG5273::getIntMode()
{
    uint16_t intModeReg = 0;
    intModeReg = readRegister(TMAG5273_INT_CONFIG_1);

    uint8_t intCon2 = bitRead(intModeReg, 2);
    uint8_t intCon3 = bitRead(intModeReg, 3);
    uint8_t intCon4 = bitRead(intModeReg, 4);

    if((intCon2 == 0) && (intCon3 == 0) && (intCon4 == 0)) // 0b000
    {
        return 0; // no interrupt
    }
    else if((intCon2 == 1) && (intCon3 == 0) && (intCon4 == 0)) // 0b001
    {
        return 1; // interrupt through !INT
    }
    else if((intCon2 == 0) && (intCon3 == 1) && (intCon4 == 0)) // 0b010
    {
        return 2; // Interrupt through !INT except when I2C is busy
    }
    else if((intCon2 == 1) && (intCon3 == 1) && (intCon4 == 0)) // 0b011
    {
        return 3; // Interrupt through SCL
    }
    else if((intCon2 == 0) && (intCon3 == 0) && (intCon4 == 1)) // 0b100
    {
        return 4; // Interrupt through SCL except when I2C is busy
    }
    else
    {
        return 0; // default
    }
}


/* SET MASK INT BIT
    This function configures the Mask !INT pin when !INT
    is connected to GND
    - 0X0 = !INT pin is enabled
    - 0X1 = !INT pin is disabled (for wake-up and trigger functions)
    TMAG5273_INT_CONFIG_1 - bit 0
*/
uint16_t TMAG5273::getMaskInt()
{
    uint16_t maskIntReg = 0;
    maskIntReg = readRegister(TMAG5273_INT_CONFIG_1);

    uint8_t maskInt = bitRead(maskIntReg, 0);

    return maskInt; 
}



/************************************************************************************************/
/************************         MAGNETIC/TEMPERATURE FUNCTIONS         ************************/
/************************************************************************************************/


/* READ TEMPERATURE
  - Enable temperature bit from T_CONFIG Register
  - Read temperature back
*/
float TMAG5273::readTemperature(void)
{

}