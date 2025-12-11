/*
 * ---------------------------------------------------------------------------------
 * Copyright (c) 2025, SparkFun Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 * ---------------------------------------------------------------------------------
 */

/*
 * Example 2: Interrupts
 * -------------------------
 * This example demonstrates how to configure the TMAG5273 sensor to use interrupts. The example is
 *  setup to trigger an interrupt when the X-axis magnetic field exceeds a specified threshold.
 *
 * * Hardware Connections:
 *.  - None reqquired -  this example is intended to be used with the SparkFun IoT Brushless Motor Driver
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

// Interrupt pin used
// NOTE: This pin is specific to the SparkFun IoT Brushless Motor Driver
uint8_t intPin = 4;

// Start the flag as false
volatile bool interruptFlag = false;

// ISR to print what interrupt was triggered
void isr1()
{
    interruptFlag = true;
}

void setup()
{
    delay(1000);
    Wire.begin();
    // Start serial communication at 115200 baud
    Serial.begin(115200);

    // Configure Interrupt Pin
    pinMode(intPin, INPUT);
    // Attach interrupt to pin 4 as a digital, falling pin
    attachInterrupt(digitalPinToInterrupt(intPin), isr1, CHANGE);

    // Begin example of the magnetic sensor code (and add whitespace for easy reading)
    Serial.println("");
    Serial.println("------------------------------------------------------------------");
    Serial.println("TMAG5273 Example 2: Interrupts");
    Serial.println("------------------------------------------------------------------");

    Serial.println("");
    // If begin is successful (0), then start example
    if (sensor.begin(i2cAddress, Wire) == true)
    {
        Serial.println("Begin");
    }
    else // Otherwise, infinite loop
    {
        Serial.println("Device failed to setup - Freezing code.");
        while (1)
            ; // Runs forever
    }

    // Set interrupt through !INT
    sensor.setInterruptMode(TMAG5273_INTERRUPT_THROUGH_INT);

    // Set the !INT pin state - pulse for 10us
    sensor.setIntPinState(true);

    // Enable the interrupt response for the thresholds
    sensor.setThresholdEn(true);

    // int pinStatus = sensor.getInterruptPinStatus();
    pinMode(intPin, INPUT);

    // Set X, Y, Z, and T Thresholds for interrupt to be triggered
    sensor.setXThreshold(5); // mT
    // sensor.setYThreshold(5);            // mT
    // sensor.setZThreshold(5);            // mT
    // sensor.setTemperatureThreshold(50);  // C

    Serial.print("X Threshold Set: ");
    Serial.println(sensor.getXThreshold());
}

/* To use the other thresholds, simply change the names and use the other functions:
- sensor.getYThreshold();
- sensor.getZThreshold();
- sensor.getTemperatureThreshold();
*/

void loop()
{
    if (interruptFlag == true)
    {
        interruptFlag = false;
        Serial.print("X Threshold Reached!  ");

        int xThresh = sensor.getXThreshold();
        Serial.print("X Threshold: ");
        Serial.println(xThresh);

        if (sensor.getMagneticChannel() != 0) // Checks if mag channels are on - turns on in setup
        {
            float magX = sensor.getXData();
            float magY = sensor.getYData();
            float magZ = sensor.getZData();
            float temp = sensor.getTemp();

            // Serial.print("(");
            // Serial.print(magX);
            // Serial.print(", ");
            // Serial.print(magY);
            // Serial.print(", ");
            // Serial.print(magZ);
            // Serial.println(") mT");
            // Serial.print(temp);
            // Serial.println(" °C");

            // This examples is primarily intended for use with the SparkFun IoT Brushless Motor Driver
            // board, which is ESP32 based. Because of this, we are using Serial.printf for easier
            // formatted output. If you are using an AVR based board, you can replace this with the
            // commented out Serial.print statements above.
            Serial.printf("     Magnetic: [ X: %7.2f  Y: %7.2f  Z: %7.2f ] mT,    Temp:\t%6.2f C\r\n", magX, magY, magZ,
                          temp);
        }
        else
        {
            Serial.println("Mag Channels disabled, stopping..");
            while (1)
                ;
        }
    }
    else
    {
        Serial.println("Checking for Interrupts...");
    }

    delay(500);
}
