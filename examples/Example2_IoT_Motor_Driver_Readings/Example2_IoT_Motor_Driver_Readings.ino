#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273.h" // Used to send and recieve specific information from our sensor
#include <SimpleFOC.h>

TMAG5273 sensor; // Initialize hall-effect sensor

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(8);

// BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(16,17, 18,23, 19,33, 34); //, 32

//target variable
float target_velocity = 5;

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


// Setup pins for current sense outputs
int curr_sense = 32;
int u_sense = 35;
int v_sense = 36;
int w_sense = 39;



void setup() 
{
  Wire.begin();
  Serial.begin(115200);   // Start serial communication at 115200 baud
  Wire.setClock(400000);  // Set clock speed to be the fastest for better communication 

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

  // Set the deadzone - duty cycle that is reserved in between changing the active MOSFET
  //driver.dead_zone(0.05); // This is injected when there is an activation/deactivation of the low/high side is used
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  Serial.print("Driver init check"); // delete once done debugging
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
  
  // Begin example of the magnetic sensor code
  Serial.println("TMAG5273 Example 1: Basic Readings");
  // If begin is successful (0), then start example
  if(sensor.begin(0X0D, Wire) == false)
  {
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    //while(1); // Runs forever
  }

  Serial.println("blah");

  int magCheck = 0;
  // Enable all magnetic channels
  magCheck = sensor.setMagChannel(7);
  Serial.print("Mag Check: ");
  Serial.println(magCheck);
  int magChannel = sensor.getMagChannel();
  Serial.println(magChannel);

  // Set the I2C Read Mode to be standard 3-byte command
  sensor.setReadMode(0);


}



void loop() 
{

  // Button Press ISR
  if(aux1.pressed){ // Change speed
    Serial.printf("Button 1 has been pressed %u times\n", aux1.numberKeyPresses);
    aux1.pressed = false;
  }
  
  if(aux2.pressed){ // RGB LED Change
    Serial.printf("Button 2 has been pressed %u times\n", aux2.numberKeyPresses);
    aux2.pressed = false;
  }


  // Basic motor movement
  motor.move(target_velocity);

  // user communication
  command.run();



  // Hall Effect Code
  // Do we need dataReady? Do we have that option? --> Look into more


  if((sensor.getMagChannel() != 0) || (sensor.getMagChannel() == -1)) // Checks if mag channels are on - turns on in setup
  {
    float magX = sensor.getXData();
    float magY = sensor.getYData();
    float magZ = sensor.getZData();
    float temp = sensor.getTemp();

    Serial.println(); // Create a whitespace for easy viewing
    Serial.print("Magnetic Field, X in mT: ");
    Serial.println(magX);
    Serial.print("Magnetic Field, Y in mT: ");
    Serial.println(magY);
    Serial.print("Magnetic Field, Z in mT: ");
    Serial.println(magZ);
    Serial.print("Temperature in Celsius: ");
    Serial.println(temp);
    delay(500); // Delay added for easier readings
    
  }
  else
  {
    Serial.println("Mag Channels disabled, stopping..");
    while(1);
  }
}
