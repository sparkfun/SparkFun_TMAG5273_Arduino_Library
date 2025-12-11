#include "SparkFun_TMAG5273_Arduino_Library.h" // Used to send and recieve specific information from our sensor
#include <Wire.h>                              // Used to establish serial communication on the I2C bus

// To download the Arduino Simple FOC Library, use this link:
// https://docs.simplefoc.com/arduino_simplefoc_library_showcase
#include <SimpleFOC.h>

/* For easy reading with this example, try opening the serial plotter! */

// Initialize hall-effect sensor
TMAG5273 sensor;

// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(8);

// BLDCDriver6PWM(int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(16, 17, 18, 23, 19, 33, 34);

// target variable in rad/s (radians per second)
float target_velocity_rps = 3;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd)
{
    command.scalar(&target_velocity_rps, cmd);
}
void doLimit(char *cmd)
{
    command.scalar(&motor.voltage_limit, cmd);
}

// Use a counter to only report every "N" loops. Otherwise data output is overwhelming!
int report_counter = 0;
int report_interval = 80; // report every 20 loops
void setup()
{
    delay(1000);
    Wire.begin();
    // Start serial communication at 115200 baud
    Serial.begin(115200);

    // Begin example of the magnetic sensor code
    Serial.println("");
    Serial.println("------------------------------------------------------------------");
    Serial.println("TMAG5273 Example 5: Basic Readings with the IoT Motor Driver");
    Serial.println("------------------------------------------------------------------");
    Serial.println("");

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
    motor.voltage_limit = 4; // [V]
    // open loop control config
    motor.controller = MotionControlType::velocity_openloop;
    // init motor hardware
    motor.init();

    // add target command T
    command.add('T', doTarget, "target velocity");
    command.add('L', doLimit, "voltage limit");

    Serial.println("Motor ready!");
    Serial.print("Set target velocity [rad/s]: ");
    Serial.println(target_velocity_rps);
    _delay(1000);

    // If the TMAG5273begin is successful (1), then start example
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
    // start our report counter at 0
    report_counter = 0;
}

void loop()
{
    // Basic motor movement
    motor.move(target_velocity_rps);

    // user communication
    command.run();

    // Hall Effect Code
    if (sensor.getMagneticChannel() != 0) // Checks if mag channels are on - turns on in begin()
    {
        if (report_counter == 0)
        {

            // get the current values
            float magX = sensor.getXData();
            float magY = sensor.getYData();
            float magZ = sensor.getZData();
            float temp = sensor.getTemp();

            // Serial.print("Data -  Magnetic: [ X:\t");
            // Serial.print(magX);
            // Serial.print("  Y:\t");
            // Serial.print(magY);
            // Serial.print("  Z:\t");
            // Serial.print(magZ);
            // Serial.print(" ] mT,    Temp:\t");
            // Serial.print(temp);
            // Serial.println(" C");

            // This examples is primarily intended for use with the SparkFun IoT Brushless Motor Driver
            // board, which is ESP32 based. Because of this, we are using Serial.printf for easier
            // formatted output. If you are using an AVR based board, you can replace this with the
            // commented out Serial.print statements above.
            Serial.printf("Magnetic: [ X: %7.2f  Y: %7.2f  Z: %7.2f ] mT,    Temp:\t%6.2f C\r\n", magX, magY, magZ,
                          temp);
        }
        // increment our counter - use the modulo operator to wrap it around
        report_counter = (report_counter + 1) % report_interval;
    }
    else
    {
        Serial.println("Mag Channels disabled, stopping..");
        while (1)
            ;
    }
}
