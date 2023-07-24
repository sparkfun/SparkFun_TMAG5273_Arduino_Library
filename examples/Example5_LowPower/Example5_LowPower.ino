#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273.h" // Used to send and recieve specific information from our sensor

TMAG5273 sensor; // Initialize hall-effect sensor

// I2C default address
uint8_t i2cAddress = I2C_ADDRESS_INITIAL;

// Interrupt pin used
// NOTE: This pin is specific to the SparkFun IoT Motor Driver
uint8_t intPin = 4;

// Start the flag as false
volatile bool interruptFlag = false;

// Sleeptime constant
int sleeptime = 0x1;

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
  // Set clock speed to be the fastest for better communication 
  Wire.setClock(1000000);  

  // Configure Interrupt Pin
  pinMode(intPin, INPUT);
  // Attach interrupt to pin 4 as a digital, falling pin
  attachInterrupt(digitalPinToInterrupt(intPin), isr1, FALLING);

  // Begin example of the magnetic sensor code (and add whitespace for easy reading)
  Serial.println("TMAG5273 Example 5: Low Power / Wake-Up and Sleep");
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

  // Setup delay for race case
  delay(500);
  
  // Set operating mode
  sensor.setOperatingMode(3); // 0X3

  // Set interrupt through !INT
  sensor.setInterruptMode(INTERRUPT_THROUGH_INT);

  // Set the !INT pin state - low until cleared
  sensor.setIntPinState(0);

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
    interruptFlag == false;

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
  sensor.setOperatingMode(3);

  delay(500);

}
