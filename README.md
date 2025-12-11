
![SparkFun Linear 3D Hall-effect Snsor (Qwiic))](docs/images/gh-banner-arduino-tmag5273.png "SparkFun Linear 3D Hall-effect Sensor")

# SparkFun TMAG5273 Arduino Library

This is the SparkFun Arduino library for the Texas Instruments TMAG5273 low-power linear 3D Hall-effect sensor with I<sup>2</sup>C interface.

![GitHub License](https://img.shields.io/github/license/sparkfun/SparkFun_TMAG5273_Arduino_Library)
![Release](https://img.shields.io/github/v/release/sparkfun/SparkFun_TMAG5273_Arduino_Library)
![Release Date](https://img.shields.io/github/release-date/sparkfun/SparkFun_TMAG5273_Arduino_Library)
![Documentation - build](https://img.shields.io/github/actions/workflow/status/sparkfun/SparkFun_TMAG5273_Arduino_Library/build-deploy-ghpages.yml?label=doc%20build)
[![Compile Test](https://github.com/sparkfun/SparkFun_FPC2534_Arduino_Library/actions/workflows/test-compile-sketch.yml/badge.svg)](https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library/actions/workflows/test-compile-sketch.yml)
![GitHub issues](https://img.shields.io/github/issues/sparkfun/SparkFun_TMAG5273_Arduino_Library)

This SparkFun Arduino Library for the Texas Instruments TMAG5273 linear 3D Hall-effect sensor provides access to the core functionality of the sensor. The sensor itself includes three Hall-effect sensors, providing readings around the X, Y and Z axes of the device.

The Texas Instruments TMAG5273 is available on the following SparkFun breakout development boards:

- [SparkFun IoT Brushless Motor Driver (ESP32 WROOM, TMC6300)](https://www.sparkfun.com/sparkfun-iot-brushless-motor-driver-esp32-wroom-tmc6300.html#content-documentation)
- [SparkFun Linear 3D Hall-Effect Sensor - TMAG5273 (Qwiic)](https://www.sparkfun.com/sparkfun-linear-3d-hall-effect-sensor-tmag5273-qwiic.html)
- [SparkFun Mini Linear 3D Hall-Effect Sensor - TMAG5273 (Qwiic)](https://www.sparkfun.com/sparkfun-mini-linear-3d-hall-effect-sensor-tmag5273-qwiic.html)

## Functionality

 SparkFun Arduino Library for the TMAG5273 provides access to the a majority of the functionality provided by the the TMAG5273 device. This includes:

- Access to X, Y and Z Hall-effect sensor readings
- Data digitized via an integrated 12-bit ADC
- Integrated temperature sensor
- Configurable to enable any combination of magnetic axes and temperature measurements
- Integrated angle calculation engine (CORDIC) provides full 360 degree position information
- Magnetic gain and offset correction settings
- Configurable I2C address, with a default value of `0x22`

### Implemented, but not supported functionality

The TMAG5273 supports several advance features that this library does not support. These are namely:

 | Feature | Notes |
 | -- | -- |
 |CRC Communication Checks|  This library contains a method to enable and disable this feature on the device, but the I2C communication implemented by the library will not read or check any provided CRS values|
 | I2C Read/Write Methods | The TMAG5273 supports different format options for I2C operations. While this library provides a method to change the I2C configuration on the device, this library only uses standard I2C operations|

 While the library include configuration methods for the above features, these features are not supported. The methods are maintained only to provide backward compatibility with earlier versions (< 2.0) of this library.

## Notes on Library Versions 2.0 and Above

Version 2.0 of this library involved a large "refactor" of the library to address existing issues, improve the overall implementation and form a foundation for easier library maintenance in the future.  As part of this effort, several existing library methods were modified such that they are not backward compatible with earlier (1.*) library versions. These changes include the following:

- The method ```readWakeUpAndSleepData()``` no longer takes pointers as parameters. References are used to simplify use.
- Various methods that used `float` type parameters and return values, now use `int8_t` types to reflect the actual sensor data.
- Equations for data values, sensor temperature and offsets were validated and updated based on the TMAG5273 datasheet. Key intermediate data typing was also validated as needed to provide correct results.

## Using the Library

The first step is to add the class named `TMAG5273` to the implementation. An example of how to declare the sensor object is as follows:

```c++

TMAG5273 sensor; // Declare the hall-effect sensor

```

In the startup section of the implementation, the sensor is initialized by calling the ```begin()``` method with the address of the sensor and the Arduino ```Wire``` port to use.

```c++
// If begin is successful (0), then start example
    if (sensor.begin(TMAG5273_I2C_ADDRESS_INITIAL, Wire) == 1)
    {
        Serial.println("Sensor Detected and Initialized");
    }
    else 
    {
        Serial.println("Sensor failed to start!");
    }
```

Note: If using the defaults, the begin method can be called with no parameters.

At this point the sensor can be configured for your specific use.

Values from the sensor are read as follows:

```c++
    // read data values
    float magX = sensor.getXData();
    float magY = sensor.getYData();
    float magZ = sensor.getZData();
    float temp = sensor.getTemp();

    // output the values to the serial console
    Serial.print("Data -  Magnetic: [ X: ");
    Serial.print(magX);
    Serial.print(", Y: ");
    Serial.print(magY);
    Serial.print(", Z: ");
    Serial.print(magZ);
    Serial.print(" ] mT,    Temp: ");
    Serial.print(temp);
    Serial.println(" C");
```

## Documentation

|Reference | Description |
|---|---|
|[Quick Reference](https://docs.sparkfun.com/SparkFun_TMAG5273_Arduino_Library/class_t_m_a_g5273.html)| A quick reference API for the the main library object |
|[Full Documentation](https://docs.sparkfun.com/SparkFun_TMAG5273_Arduino_Library/)| The full documentation and API for this Arduino library|
|[SparkFun Hall-Effect Sensor - TMAG5273](https://github.com/sparkfun/SparkFun_Qwiic_Hall_Effect_Sensor_TMAG5273)| Hardware GitHub Repository|
|[Hall-Effect Sensor Hook Up Guide](https://docs.sparkfun.com/SparkFun_Qwiic_Hall_Effect_Sensor_TMAG5273) | Hardware Overview and Quick Start for the SparkFun Hall-Effect Sensor - TMAG5273 |
|[SparkFun IoT Brushless Motor Driver (ESP32 WROOM, TMC6300)](https://github.com/sparkfun/SparkFun_IoT_Brushless_Motor_Driver)| Hardware GitHub Repository|
|[SparkFun IoT Brushless Motor DriverHook Up Guide](https://docs.sparkfun.com/SparkFun_IoT_Brushless_Motor_Driver/introduction/) | Hardware Overview and Quick Start for the SparkFun IoT Brushless Motor Driver |

## Examples

The following examples are provided with the library

| Example | Description |
|---|---|
| [Basic Readings](examples/Example1_BasicReadings/Example1_BasicReadings.ino) | Shows basic sensor setup and value reading|
| [Interrupt Triggered Reading](examples/Example2_Interrupts/Example2_Interrupts.ino)| Demostraights how to use a interrupt  to trigger data reading|
| [Angle Calculations](examples/Example3_AngleCalculations/Example3_AngleCalculations.ino) | Setting up and reading Angle Calculations from the sensor|
| [I2C Address Setting](examples/Example4_I2CSettings/Example4_I2CSettings.ino) | How to change the I2C address of the sensor|
| [IOT Motor Driver Readings](examples/Example5_IoT_Motor_Driver_Readings/Example5_IoT_Motor_Driver_Readings.ino) | Using the TMAG5273 with the SparkFun IoT Brushless Motor development board|

## License Information

This product is ***open source***!

This product is licensed using the [MIT Open Source License](https://opensource.org/license/mit).
