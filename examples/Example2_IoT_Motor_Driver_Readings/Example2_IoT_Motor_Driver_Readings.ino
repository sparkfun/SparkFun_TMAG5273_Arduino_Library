#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273_Arduino_Library.h" // Used to send and recieve specific information from our sensor
#include <SimpleFOC.h>

/* For easy reading with this example, try opening the serial plotter! */

// Initialize hall-effect sensor
TMAG5273 sensor; 

// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(8);

// BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(16,17, 18,23, 19,33, 34);

//target variable
float target_velocity = 3;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

// Checkout peripherals (aux buttons, leds, etc)
int aux1Pin = 14;
int aux2Pin = 13;
int ledPin = 2;

// Use as the current pin in test for lighting LED test circuit
int testPin = 12;

// Creates a class for the button information to be stored in for the ISR
struct Button{
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

// Creates an instance of each button and their pin on the ESP32
Button aux1 = {14, 0, false};
Button aux2 = {13, 0, false};

// Button 1 ISR to increment button count and update status of press
void IRAM_ATTR isr1(){
  aux1.numberKeyPresses++;
  aux1.pressed = true;
}

// Button 2 ISR to increment button count and update status of press
void IRAM_ATTR isr2(){
  aux2.numberKeyPresses++;
  aux2.pressed = true;
}


void setup() 
{  
  Wire.begin();
  // Start serial communication at 115200 baud
  Serial.begin(115200);   
  // Set clock speed to be the fastest for better communication
  Wire.setClock(1000000);   

  // Begin example of the magnetic sensor code
  Serial.println("TMAG5273 Example 2: Basic Readings with the IoT Motor Driver");

  // ********* Driver config *********
  // power supply voltage [V]
  driver.voltage_power_supply = 3.3; // ESP32 3.3V supply pin
  // PWM Configuration
  driver.pwm_frequency = 20000;
  // this value is fixed on startup
  driver.voltage_limit = 4;
  driver.init();
  // link the motor and the driver to enable usage
  motor.linkDriver(&driver);
  // limit the voltage to be set to the motor
  motor.voltage_limit = 4;   // [V]
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;
  // init motor hardware
  motor.init();
 
  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.print("Set target velocity [rad/s]: ");
  Serial.println(target_velocity);
  _delay(1000);

  // Making RGB LED an output
  pinMode(ledPin, OUTPUT); // RGB LED

  // Aux pins as an interrupt
  pinMode(aux1.PIN, INPUT_PULLUP); // Sets pin 14 on the ESP32 as an interrupt
  attachInterrupt(aux1.PIN, isr1, FALLING); // Triggers when aux1 is pulled to GND (button pressed)
  pinMode(aux2.PIN, INPUT_PULLUP); // Sets pin 13 on the ESP32 as an interrupt
  attachInterrupt(aux2.PIN, isr2, FALLING); // Triggers when aux2 is pulled to GND (button pressed)
  

  // If begin is successful (1), then start example
  if(sensor.begin(i2cAddress, Wire) == true)
  {
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    while(1); // Runs forever
  }

  // Delay for race case setup
  delay(500);
}


void loop() 
{
  // Variable for temperature 
  float temp = sensor.getTemp();
  // Button Press ISR
  if(aux1.pressed)
  { 
    Serial.print("Temperature: ");
    Serial.println(temp);
    aux1.pressed = false;
    delay(1000);
  }
  if(aux2.pressed)
  { 
    Serial.println("Changing direction.. ");
    aux2.pressed = false;
  }

  // Basic motor movement
  motor.move(target_velocity);

  // user communication
  command.run();

  // Hall Effect Code
  if(sensor.getMagneticChannel() != 0) // Checks if mag channels are on - turns on in begin()
  {
    float magX = sensor.getXData();
    float magY = sensor.getYData();
    float magZ = sensor.getZData();

    Serial.print("(");
    Serial.print(magX);
    Serial.print(", ");
    Serial.print(magY);
    Serial.print(", ");
    Serial.print(magZ);
    Serial.println(") mT");
  }
  else
  {
    Serial.println("Mag Channels disabled, stopping..");
    while(1);
  }

}
