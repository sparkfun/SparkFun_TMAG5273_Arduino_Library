#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273_Arduino_Library.h" // Used to send and recieve specific information from our sensor

TMAG5273 sensor; // Initialize hall-effect sensor

// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

// Interrupt pin used
// NOTE: This pin is specific to the SparkFun IoT Motor Driver
uint8_t intPin = 4;

// Start the flag as false
volatile bool interruptFlag = false;

// Sleeptime constant
int sleeptime = TMAG5273_SLEEP_5MS;

// ISR to print what interrupt was triggered
void isr1() 
{
  interruptFlag = true;
}


void setup() 
{
  Wire.begin();
  // Start serial communication at 115200 baud
  Serial.begin(115200);   


  // Begin example of the magnetic sensor code (and add whitespace for easy reading)
  Serial.println("TMAG5273 Example 3: Low Power / Wake-Up and Sleep");
  Serial.println("");
  // If begin is successful (0), then start example
  if(sensor.begin(i2cAddress, Wire) == true)
  {
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    while(1); // Runs forever
  }

  // Configure Interrupt Pin
  pinMode(intPin, OUTPUT);
  // Attach interrupt to pin 4 as a digital, falling pin
  attachInterrupt(digitalPinToInterrupt(intPin), isr1, FALLING);
  
  // Set operating mode to wake up and sleep
  sensor.setOperatingMode(TMAG5273_WAKE_UP_AND_SLEEP_MODE); // 0X3

  // Set interrupt through !INT
  sensor.setInterruptMode(TMAG5273_INTERRUPT_THROUGH_INT);

  // Set the !INT pin state - low until cleared
  sensor.setIntPinState(false);

  // Set time spent in low power mode between conversions
  sensor.setSleeptime(sleeptime);

  int opMode = sensor.getOperatingMode();
  Serial.print("Operating Mode set: "); // Debug purpose
  Serial.println(opMode);

}


void loop()
{ 
  // Initialize values
  float magX = 0;
  float magY = 0;
  float magZ = 0;
  float temperatureData = 0;

  // If the interrupt was triggered, read from sleep mode
  if(interruptFlag == true)
  {
    // Assert flag back to false
    interruptFlag = false;

    if(sensor.readWakeUpAndSleepData(&magX, &magY, &magZ, &temperatureData) == 0)
    {
      Serial.print("(");
      Serial.print(magX);
      Serial.print(", ");
      Serial.print(magY);
      Serial.print(", ");
      Serial.print(magZ);
      Serial.println(") mT");

      Serial.print(temperatureData);
      Serial.println(" C");
      // Empty space for easier reading
      Serial.println();
    }

  }

  // Set operating mode back to wakeup and sleep
  sensor.setOperatingMode(TMAG5273_WAKE_UP_AND_SLEEP_MODE);

  delay(500);

}
