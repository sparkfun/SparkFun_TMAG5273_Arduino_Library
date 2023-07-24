#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273.h" // Used to send and recieve specific information from our sensor

// Create a new sensor object
TMAG5273 sensor; 

// I2C default address
uint8_t i2cAddress = I2C_ADDRESS_INITIAL;

void setup() 
{
  Wire.begin();
  // Start serial communication at 115200 baud
  Serial.begin(115200);   
  // Set clock speed to be the fastest for better communication 
  Wire.setClock(1000000);  

  // Begin example of the magnetic sensor code (and add whitespace for easy reading)
  Serial.println("TMAG5273 Example 3: Angle Calculations");
  Serial.println("");
  // If begin is successful (0), then start example
  if(sensor.begin(0X22, Wire) == true)
  {
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    while(1); // Runs forever
  }

  // Set the device at 32x average mode 
  sensor.setConvAvg(X32_CONVERSION);

  // Choose new angle to calculate from
  // Can calculate angles between XYX, YXY, YZY, and XZX
  sensor.setMagneticChannel(XYX_ENABLE);

  // Enable the angle calculation register
  // Can choose between XY, YZ, or XZ priority
  sensor.setAngleEn(XY_ANGLE_CALCULATION);

}


void loop() 
{
  if((sensor.getMagneticChannel() != 0) && (sensor.getAngleEn() != 0)) // Checks if mag channels are on - turns on in setup
  {
    float angleCalculation = sensor.getAngleResult();

    Serial.print("XYX: ");
    Serial.print(angleCalculation);
    Serial.println(" mT");

  }
  else
  {
    Serial.println("Mag Channels disabled, stopping..");
    while(1);
  }

  delay(1000);
}
