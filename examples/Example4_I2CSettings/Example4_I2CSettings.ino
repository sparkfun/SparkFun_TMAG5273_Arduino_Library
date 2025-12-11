/*
 * ---------------------------------------------------------------------------------
 * Copyright (c) 2025, SparkFun Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 * ---------------------------------------------------------------------------------
 */

/*
 * Example 4: I2C Settings - Address Change
 * -------------------------
 * This example demonstrates how to change the I2C address of the TMAG5273 sensor.
 *
 * * Hardware Connections:
 * - Connect the TMAG5273 sensor to the SparkFun Qwiic connector on your SparkFun microcontroller board.
 *
 * Note: Make sure to install the SparkFun TMAG5273 Arduino Library before running this example.
 * You can install it via the Arduino Library Manager or download it from:
 *      https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library
 *
 */
#include "SparkFun_TMAG5273_Arduino_Library.h" // Used to send and recieve specific information from our sensor
#include <Wire.h>                              // Used to establish serial communication on the I2C bus

TMAG5273 sensor; // Initialize hall-effect sensor

// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;
uint8_t updatedAddress = TMAG5273_I2C_ADDRESS_INITIAL + 1; // New address to set

void setup()
{
    delay(1000);
    Wire.begin();
    // Start serial communication at 115200 baud
    Serial.begin(115200);

    // Begin example of the magnetic sensor code (and add whitespace for easy reading)
    Serial.println("");
    Serial.println("------------------------------------------------------------------");
    Serial.println("TMAG5273 Example 4: I2C Settings - Address Change");
    Serial.println("------------------------------------------------------------------");
    Serial.println("");

    Serial.println("Startup:");
    Serial.println("");
    Serial.print("  - Startup sketch defined address: ");
    Serial.println(i2cAddress, HEX);

    // If begin is successful (1), then start example
    if (sensor.begin(i2cAddress, Wire) == true)
    {
        Serial.println("  - Connected to TMAG5273 sensor.");
    }
    else // Otherwise, infinite loop
    {
        Serial.println("Device failed to setup - Freezing code.");

        while (1)
            ; // Runs forever
    }

    Serial.print("  - Address read from Sensor: ");
    Serial.println(sensor.getI2CAddress(), HEX);
    if (sensor.getI2CAddress() != i2cAddress)
    {
        Serial.println("Address mismatch! - Device might required setting address to default");
        Serial.println("Freezing code.");
        while (1)
            ;
    }
    delay(500);

    // Change I2C Address - must be 7-bits (bits 1-7)
    // NOTE: The I2C address will reset back to default (0x22) after a power cycle.
    // Running other examples without a power cycle will fail due to the I2C address being different.
    Serial.println("");
    Serial.println("Change I2C Address:");
    Serial.println("");
    Serial.print("  - Setting device I2C Address to: ");
    Serial.println(updatedAddress, HEX);

    // Set the I2C Address
    if (sensor.setI2CAddress(updatedAddress) != 0)
    {
        Serial.println("Address change call failed - freezing code.");
        while (1)
            ;
    }
    delay(100);
    Serial.print("  - Address from Sensor: ");
    Serial.println(sensor.getI2CAddress(), HEX);
    if (sensor.getI2CAddress() != updatedAddress)
    {
        Serial.println("Address mismatch after set! - Freezing code.");
        while (1)
            ;
    }

    delay(500);
    Serial.println("");
    // Now lets set the the address back to the original for future example
    Serial.println("Restore I2C Address:");
    Serial.println("");
    Serial.print("  - Setting I2C Address to: ");
    Serial.println(i2cAddress, HEX);
    if (sensor.setI2CAddress(i2cAddress) != 0)
    {
        Serial.println("Address change call failed - freezing code.");
        while (1)
            ;
    }
    delay(100);
    Serial.print("  - Address from Sensor: ");
    Serial.println(sensor.getI2CAddress(), HEX);
    if (sensor.getI2CAddress() != i2cAddress)
    {
        Serial.println("Address mismatch after restore! - Freezing code.");
        while (1)
            ;
    }
    Serial.println("");
    Serial.println("I2C Address change example complete.");
    Serial.println("------------------------------------------------------------------");
    Serial.println("");
}

void loop()
{
    // nothing ...
    // Wait 1 second for next scan
    delay(3000);
}