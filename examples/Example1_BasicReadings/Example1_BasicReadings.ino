#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273.h" // Used to send and recieve specific information from our sensor

TMAG5273 sensor; // Initialize hall-effect sensor

void setup() 
{
  Wire.begin();
  // Start serial communication at 115200 baud
  Serial.begin(115200);   
  // Set clock speed to be the fastest for better communication 
  Wire.setClock(1000000);  

  
  // Begin example of the magnetic sensor code (and add whitespace for easy reading)
  Serial.println("TMAG5273 Example 1: Basic Readings");
  Serial.println("");
  // If begin is successful (0), then start example
  if(sensor.begin(0X22, Wire) == false)
  {
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    while(1); // Runs forever
  }

  // TESTING CODE: 
  // Print out the I2C address
  uint16_t address = sensor.getI2CAddress();
  Serial.print("I2C Address: ");
  Serial.println(address, HEX);
  
  sensor.setMagChannel(7);
  int magChannel = sensor.getMagChannel();
  Serial.print("Magnetic Channel set: ");
  Serial.println(magChannel);

  sensor.setXYAxisRange(1);

}



void loop() 
{

  if(sensor.getMagChannel() != 0) // Checks if mag channels are on - turns on in setup
  {
    float magX = sensor.getXData();
    float magY = sensor.getYData();
    float magZ = sensor.getZData();
    float temp = sensor.getTemp();
 
    int xyField = sensor.getXYAxisRange();
    int xLSB = sensor.getXLSB();
    int xMSB = sensor.getXMSB();
    Serial.println(); // Create a whitespace for easy viewing
    Serial.print("Magnetic Field, X : ");
    Serial.print(magX);
    Serial.println("mT");
    Serial.print("XY Axis Range: ");
    Serial.println(xyField);
    Serial.print("XLSB: ");
    Serial.println(xLSB, BIN);
    Serial.print("XMSB: ");
    Serial.println(xMSB, BIN);
    //Serial.print("Magnetic Field, Y in mT: ");
    //Serial.println(magY);
    //Serial.print("Magnetic Field, Z in mT: ");
    //Serial.println(magZ);
    //Serial.print("Temperature in Celsius: ");
    //Serial.println(temp);
    delay(500); // Delay added for easier readings
  }
  else
  {
    Serial.println("Mag Channels disabled, stopping..");
    while(1);
  }

  delay(1000);
}
