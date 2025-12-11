/******************************************************************************
SparkFun_TMAG5273_Arduino_Library_Defs.h
SparkFunTMAG5273 Library Definitions Header File
Madison Chodikov @ SparkFun Electronics
Original Creation Date: May 1st, 2023
https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library

This file defines all of the constants used in the device library,
along with defining the registers used.

Development environment specifics:
    IDE: Arduino 2.1.0
    Hardware Platform: Arduino Uno
    TMAG5273 Breakout Version: 1.0.1
Distributed as-is; no warranty is given.
******************************************************************************/

/*
Features as per datasheet
    - ±40mT on the X, Y linear magnetic range
        - 820 LSB/mT Sensitivity
    - ±80mT on the Z linear magnetic range
        - 410 LSB/mT Sensitivity
    - ±300 µT Offset
        - ±3 µT Offset Drift
*/

#ifndef __SparkFun_TMAG5273_Arduino_Library_Defs_H__
#define __SparkFun_TMAG5273_Arduino_Library_Defs_H__

/********************************** Constant Variable Definitions **********************************/
#define TMAG5273_DEVICE_ID_VALUE 0x5449   // Value found in the device ID register
#define TMAG5273_I2C_ADDRESS_INITIAL 0x22 // Initial I2C address value - can be changed using functions as seen below
#define TMAG5273_TSENSE_T0 25.0f          // Reference temperature for TADC_T0
#define TMAG5273_TADC_T0 17508            // Temp result in decimal value (from 16-buit format)
#define TMAG5273_TADC_RES 60.1f           // Temperature sensing resolution (in 16-bit format)

#define TMAG5273_CRC_MODE_BITS 0x80 // Bit 7
#define TMAG5273_CRC_MODE_LSB 7
#define TMAG5273_CRC_DISABLE 0x0 // Disables I2C CRC byte to be sent
#define TMAG5273_CRC_ENABLE 0x1  // Enable I2C CRC byte to be sent

#define TMAG5273_MAG_TEMP_BITS 0x60 // Bits 6-5
#define TMAG5273_MAG_TEMP_LSB 5
#define TMAG5273_MAG_TEMP_0PCT 0x0     // 0% (No temperature compensation)
#define TMAG5273_MAG_TEMP_0P12PCT 0x1  // 0.12%/deg C (NdBFe)
#define TMAG5273_MAG_TEMP_RESERVED 0x2 // Reserved
#define TMAG5273_MAG_TEMP_0P2PCT 0x3   //

#define TMAG5273_CONV_AVG_BITS 0x1A // Bits 4-2
#define TMAG5273_CONV_AVG_LSB 2
#define TMAG5273_X1_CONVERSION 0x0  // 1X Average
#define TMAG5273_X2_CONVERSION 0x1  // 2X Average
#define TMAG5273_X4_CONVERSION 0x2  // 4X Average
#define TMAG5273_X8_CONVERSION 0x3  // 8X Average
#define TMAG5273_X16_CONVERSION 0x4 // 16X Average
#define TMAG5273_X32_CONVERSION 0x5 // 32X Average

#define TMAG5273_I2C_READ_MODE_BITS 0x03 // Bits 1-0
#define TMAG5273_I2C_READ_MODE_LSB 0
#define TMAG5273_I2C_MODE_3BYTE 0x0       // Standard I2C 3-byte read command
#define TMAG5273_I2C_MODE_1BYTE_16BIT 0x1 // 1-byte I2C read command for 16bit sensor data and conversion status
#define TMAG5273_I2C_MODE_1BYTE_8BIT 0x2  // 1-byte I2C read command for 8 bit sensor MSB data and conversion status

#define TMAG5273_THR_HYST_BITS 0xE0 // Bits 7-5
#define TMAG5273_THR_HYST_LSB 5
#define TMAG5273_THRESHOLD_HYST_2COMP 0x0 // 2's complement threshold hysteresis
#define TMAG5273_THRESHOLD_HYST_7LSB 0x1  // 7 LSB threshold hysteresis

#define TMAG5273_LOW_POWER_BITS 0x10 // Bit 4
#define TMAG5273_LOW_POWER_LSB 4
#define TMAG5273_LOW_ACTIVE_CURRENT_MODE 0x0 // Low active current mode
#define TMAG5273_LOW_NOISE_MODE 0x1          // Low noise mode

#define TMAG5273_GLITCH_FILTER_BITS 0x08 // Bit 3
#define TMAG5273_GLITCH_FILTER_LSB 3
#define TMAG5273_GLITCH_ON 0x0  // Glitch filter on
#define TMAG5273_GLITCH_OFF 0x1 // Glitch filter off

#define TMAG5273_TRIGGER_MODE_BITS 0x04 // Bit 2
#define TMAG5273_TRIGGER_MODE_LSB 2
#define TMAG5273_TRIGGER_I2C_COMMAND 0x0 // Triggered by I2C command
#define TMAG5273_TRIGGER_INT_SIGNAL 0x1  // Triggered by INT signal

#define TMAG5273_OPERATING_MODE_BITS 0x03 // Bits 1-0
#define TMAG5273_OPERATING_MODE_LSB 0
#define TMAG5273_STANDY_BY_MODE 0x0          // Stand-by mode
#define TMAG5273_SLEEP_MODE 0x1              // Sleep mode
#define TMAG5273_CONTINUOUS_MEASURE_MODE 0x2 // Continuous measure mode
#define TMAG5273_WAKE_UP_AND_SLEEP_MODE 0x3  // Wake-up and sleep mode

// Sensor Channel Modes
#define TMAG5273_CHANNEL_MODE_BITS 0xF0 // Bits 7-4
#define TMAG5273_CHANNEL_MODE_LSB 4
#define TMAG5273_CHANNELS_OFF 0x0 // Turn all the magnetic channels off
#define TMAG5273_X_ENABLE 0x1     // X Channel enabled
#define TMAG5273_Y_ENABLE 0x2     // Y Channel enabled
#define TMAG5273_X_Y_ENABLE 0x3   // X, Y Channel enabled
#define TMAG5273_Z_ENABLE 0x4     // Z Channel enabled
#define TMAG5273_Z_X_ENABLE 0x5   // Z, X Channel enabled
#define TMAG5273_Y_Z_ENABLE 0x6   // Y, Z Channel enabled
#define TMAG5273_X_Y_Z_ENABLE 0x7 // X, Y, Z Channel enabled
#define TMAG5273_XYX_ENABLE 0x8   // XYX Channel enabled
#define TMAG5273_YXY_ENABLE 0x9   // YXY Channel enabled
#define TMAG5273_YZY_ENABLE 0xA   // YZY Channel enabled
#define TMAG5273_XZX_ENABLE 0xB   // XZX Channel enabled

#define TMAG5273_SLEEP_MODE_BITS 0x0F // Bits 3-0
#define TMAG5273_SLEEP_MODE_LSB 0
#define TMAG5273_SLEEP_1MS 0x0     // 1ms
#define TMAG5273_SLEEP_5MS 0x1     // 5ms
#define TMAG5273_SLEEP_10MS 0x2    // 10 ms
#define TMAG5273_SLEEP_15MS 0x3    // 15ms
#define TMAG5273_SLEEP_20MS 0x4    // 20ms
#define TMAG5273_SLEEP_30MS 0x5    // 30ms
#define TMAG5273_SLEEP_50MS 0x6    // 50ms
#define TMAG5273_SLEEP_100MS 0x7   // 100ms
#define TMAG5273_SLEEP_500MS 0x8   // 500ms
#define TMAG5273_SLEEP_1000MS 0x9  // 1000ms
#define TMAG5273_SLEEP_2000MS 0xA  // 2000ms
#define TMAG5273_SLEEP_5000MS 0xB  // 5000ms
#define TMAG5273_SLEEP_20000MS 0xC // 20000ms

#define TMAG5273_THRESHOLD_1 0x0 // 1 Threshold crossing
#define TMAG5273_THRESHOLD_4 0x1 // 4 Threshold crossing

#define TMAG5273_THRESHOLD_INT_BITS 0x20 // Bit 5
#define TMAG5273_THRESHOLD_INT_LSB 5
#define TMAG5273_THRESHOLD_INT_ABOVE 0x0 // Sets interrupt for field above the threshold
#define TMAG5273_THRESHOLD_INT_BELOW 0x1 // Sets interrupt for field below the threshold

#define TMAG5273_GAIN_ADJUST_BITS 0x10 // bit 4
#define TMAG5273_GAIN_ADJUST_LSB 4
#define TMAG5273_GAIN_ADJUST_CHANNEL_1 0x0 // 1st channel is selected for gain adjustment
#define TMAG5273_GAIN_ADJUST_CHANNEL_2 0x1 // 2nd channel is selected for gain adjustment

#define TMAG5273_ANGLE_CALCULATION_BITS 0x0C // Bits 3-2
#define TMAG5273_ANGLE_CALCULATION_LSB 2
#define TMAG5273_NO_ANGLE_CALCULATION 0x0 // No angle calculation, magnetic gain, and offset correction enabled
#define TMAG5273_XY_ANGLE_CALCULATION 0x1 // X 1st, Y 2nd
#define TMAG5273_YZ_ANGLE_CALCULATION 0x2 // Y 1st, Z 2nd
#define TMAG5273_XZ_ANGLE_CALCULATION 0x3 // X 1st, Z 2nd

// xy axis range bigs
#define TMAG5273_XY_RANGE_BITS 0x02 // Bit 1
#define TMAG5273_XY_RANGE_LSB 1

// z axis range bits
#define TMAG5273_Z_RANGE_BITS 0x01 // Bit 0
#define TMAG5273_Z_RANGE_LSB 0

// these are the same for xy and z
#define TMAG5273_RANGE_40MT 0x0 // +/-40mT, DEFAULT
#define TMAG5273_RANGE_80MT 0x1 // +/-80mT, DEFAULT

#define TMAG5273_TEMPERATURE_BITS 0x01 // Bit 0
#define TMAG5273_TEMPERATURE_LSB 0
#define TMAG5273_TEMPERATURE_DISABLE 0x0 // Temperature channel disabled
#define TMAG5273_TEMPERATURE_ENABLE 0x1  // Temperature channel enabled

#define TMAG5273_INTERRUPT_RESULT_BITS 0x80 // Bit 7
#define TMAG5273_INTERRUPT_RESULT_LSB 7

#define TMAG5273_INTERRUPT_THRESHOLD_BITS 0x40 // Bit 6
#define TMAG5273_INTERRUPT_THRESHOLD_LSB 6

#define TMAG5273_INTERRUPT_NOT_ASSERTED 0x0 // Interrupt is not asserted when set
#define TMAG5273_INTERRUPT_ASSERTED 0x1     // Interrupt is asserted

#define TMAG5273_INTERRUPT_PIN_STATE_BITS 0x20 // Bit 5
#define TMAG5273_INTERRUPT_PIN_STATE_LSB 5

#define TMAG5273_INTERRUPT_MODE_BITS 0x1C // Bits 4-2
#define TMAG5273_INTERRUPT_MODE_LSB 2
#define TMAG5273_NO_INTERRUPT 0x0              // No interrupt
#define TMAG5273_INTERRUPT_THROUGH_INT 0x1     // Interrupt thru INT
#define TMAG5273_INTERRUPT_THROUGH_INT_I2C 0x2 // Interrupt thru INT except when I2C bus is busy
#define TMAG5273_INTERRUPT_THROUGH_SCL 0x3     // Interrupt thru SCL
#define TMAG5273_INTERRUPT_THROUGH_SCL_I2C 0x4 // Interrupt thru SCL except when I2C bus is busy

#define TMAG5273_INTERRUPT_MASK_BITS 0x01 // Bit 0
#define TMAG5273_INTERRUPT_MASK_LSB 0
#define TMAG5273_INTERRUPT_ENABLED 0x0  // Interrupt pin is enabled
#define TMAG5273_INTERRUPT_DISABLED 0x1 // Interrupt pin is disabled

#define TMAG5273_INT_PIN_ENABLE 0x0  // INT pin is enabled
#define TMAG5273_INT_PIN_DISABLE 0x1 // INT pin is disabled (for wake-up and trigger functions)

#define TMAG5273_I2C_ADDRESS_BITS 0xFE // Bits 7-1
#define TMAG5273_I2C_ADDRESS_LSB 1

#define TMAG5273_I2C_ADDRESS_CHANGE_BITS 0x00 // Bit 0
#define TMAG5273_I2C_ADDRESS_CHANGE_LSB 0
#define TMAG5273_I2C_ADDRESS_CHANGE_DISABLE 0x0 // Disable update of I2C address
#define TMAG5273_I2C_ADDRESS_CHANGE_ENABLE 0x1  // Enable update of I2C address

#define TMAG5273_NO_POR 0x0  // Conversion data not complete
#define TMAG5273_YES_POR 0x1 // Conversion data complete

#define TMAG5273_OSCILLATOR_ERROR_BITS 0x08 // Bit 3
#define TMAG5273_OSCILLATOR_ERROR_LSB 3
#define TMAG5273_OSCILLATOR_ERROR_UNDETECTED 0x0 // No Oscillator error detected
#define TMAG5273_OSCILLATOR_ERROR_DETECTED 0x1   // Oscillator error detected

#define TMAG5273_INT_ERROR_UNDETECTED 0x0 // No INT error detected
#define TMAG5273_INT_ERROR_DETECTED 0x1   // INT error detected

#define TMAG5273_CONV_STATUS_SET_COUNT_BITS 0xE0 // Bits 7-5
#define TMAG5273_CONV_STATUS_SET_COUNT_LSB 5

#define TMAG5273_CONV_STATUS_POR_BITS 0x10 // Bit 4
#define TMAG5273_CONV_STATUS_POR_LSB 4

#define TMAG5273_CONV_STATUS_DIAG_STATUS_BITS 0x02 // Bit 1
#define TMAG5273_CONV_STATUS_DIAG_STATUS_LSB 1

#define TMAG5273_CONV_STATUS_RESULT_STATUS_BITS 0x01 // Bit 0
#define TMAG5273_CONV_STATUS_RESULT_STATUS_LSB 0

#define TMAG5273_DEVICE_ID_BITS 0x03 // Bits 1-0
#define TMAG5273_DEVICE_ID_LSB 0

#define TMAG5273_DEVICE_STATUS_INTR_RB_BITS 0x10 // Bit 4
#define TMAG5273_DEVICE_STATUS_INTR_RB_LSB 4

#define TMAG5273_DEVICE_STATUS_OSCILLATOR_ERROR_BITS 0x08 // Bit 3
#define TMAG5273_DEVICE_STATUS_OSCILLATOR_ERROR_LSB 3

#define TMAG5273_DEVICE_STATUS_INT_ERROR_BITS 0x04 // Bit 2
#define TMAG5273_DEVICE_STATUS_INT_ERROR_LSB 2

#define TMAG5273_DEVICE_STATUS_OTP_CRC_ERROR_BITS 0x02 // Bit 1
#define TMAG5273_DEVICE_STATUS_OTP_CRC_ERROR_LSB 1

#define TMAG5273_DEVICE_STATUS_VCC_UV_ERROR_BITS 0x01 // Bit 0
#define TMAG5273_DEVICE_STATUS_VCC_UV_ERROR_LSB 0

// From the datasheet - section 8.2.1.2 - Settings to place the sensor in  Wakeup and Sleep Mode
// these are typed consts to support method overloading
const uint8_t TMAG5273_WAKEUP_SLEEP_MODE_INT_CONFIG_1_SETTINGS = 0x64;
const uint8_t TMAG5273_WAKEUP_SLEEP_MODE_DEVICE_CONFIG_2_SETTINGS = 0x23;

/********************************** Register Definitions **********************************/

const uint8_t TMAG5273_REG_DEVICE_CONFIG_1 = 0x00;
const uint8_t TMAG5273_REG_DEVICE_CONFIG_2 = 0x01;
const uint8_t TMAG5273_REG_SENSOR_CONFIG_1 = 0x02;
const uint8_t TMAG5273_REG_SENSOR_CONFIG_2 = 0x03;
const uint8_t TMAG5273_REG_X_THR_CONFIG = 0x04;
const uint8_t TMAG5273_REG_Y_THR_CONFIG = 0x05;
const uint8_t TMAG5273_REG_Z_THR_CONFIG = 0x06;
const uint8_t TMAG5273_REG_T_CONFIG = 0x07;
const uint8_t TMAG5273_REG_INT_CONFIG_1 = 0x08;
const uint8_t TMAG5273_REG_MAG_GAIN_CONFIG = 0x09;
const uint8_t TMAG5273_REG_MAG_OFFSET_CONFIG_1 = 0x0A;
const uint8_t TMAG5273_REG_MAG_OFFSET_CONFIG_2 = 0x0B;
const uint8_t TMAG5273_REG_I2C_ADDRESS = 0x0C;
const uint8_t TMAG5273_REG_DEVICE_ID = 0x0D;
const uint8_t TMAG5273_REG_MANUFACTURER_ID_LSB = 0x0E;
const uint8_t TMAG5273_REG_MANUFACTURER_ID_MSB = 0x0F;
const uint8_t TMAG5273_REG_T_MSB_RESULT = 0x10;
const uint8_t TMAG5273_REG_T_LSB_RESULT = 0x11;
const uint8_t TMAG5273_REG_X_MSB_RESULT = 0x12;
const uint8_t TMAG5273_REG_X_LSB_RESULT = 0x13;
const uint8_t TMAG5273_REG_Y_MSB_RESULT = 0x14;
const uint8_t TMAG5273_REG_Y_LSB_RESULT = 0x15;
const uint8_t TMAG5273_REG_Z_MSB_RESULT = 0x16;
const uint8_t TMAG5273_REG_Z_LSB_RESULT = 0x17;
const uint8_t TMAG5273_REG_CONV_STATUS = 0x18;
const uint8_t TMAG5273_REG_ANGLE_RESULT_MSB = 0x19;
const uint8_t TMAG5273_REG_ANGLE_RESULT_LSB = 0x1A;
const uint8_t TMAG5273_REG_MAGNITUDE_RESULT = 0x1B;
const uint8_t TMAG5273_REG_DEVICE_STATUS = 0x1C;

#endif
