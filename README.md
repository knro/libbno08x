# BNO08x C++ Library for Linux

This is a C++ shared library for the BNO08x IMU, based on the Adafruit BNO08x library. This library is designed to work on Linux systems.

## Dependencies

This library requires the following dependencies to be installed:

- `libgpiod`
- `cmake`
- A C++ compiler (e.g., `g++`)

On Debian-based systems, you can install these with:

```bash
sudo apt-get install libgpiod-dev g++ cmake
```

## Installation

This library is built using CMake. To build the library, run the following commands:

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

## Usage

This library supports both I2C and SPI communication with the BNO08x.

### I2C

To initialize the sensor using I2C, use the `begin_i2c` method. The default I2C address is `0x4A`, but you can specify a different address.

```cpp
#include <BNO08x.h>
#include <iostream>

int main() {
    BNO08x imu;
    // The default I2C address is 0x4A. Use 0x4B if the DI pin is pulled high.
    if (imu.begin_i2c(0x4A)) {
        std::cout << "BNO08x found!" << std::endl;
    } else {
        std::cerr << "BNO08x not found" << std::endl;
    }
    return 0;
}
```

### SPI

To initialize the sensor using SPI, use the `begin_spi` method:

```cpp
#include <BNO08x.h>
#include <iostream>

int main() {
    BNO08x imu;
    // arguments are (spi_device, cs_pin, int_pin)
    if (imu.begin_spi("/dev/spidev0.0", 5, 6)) {
        std::cout << "BNO08x found!" << std::endl;
    } else {
        std::cerr << "BNO08x not found" << std::endl;
    }
    return 0;
}
```

## Usage in a CMake Project

To use this library in your own CMake project, you need to find the package and link against it.

```cmake
find_package(BNO08x REQUIRED)
target_link_libraries(your_target PRIVATE BNO08x::bno08x)
```

Here is a complete `CMakeLists.txt` example for an executable that uses this library:

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)

find_package(BNO08x REQUIRED)

add_executable(MyExecutable main.cpp)
target_link_libraries(MyExecutable PRIVATE BNO08x::bno08x)
```

## Getting Sensor Data

To get sensor data, you first need to enable the desired sensor reports, and then continuously poll for new sensor events.

### 1. Enable Sensor Reports

Use the `enableReport` function to start receiving data from a sensor. You can enable multiple sensors. The function takes the sensor ID and an optional report interval in microseconds.

```cpp
#include "BNO08x.h"

BNO08x imu;
// ... initialize with begin_i2c or begin_spi ...

// Enable the rotation vector sensor
if (!imu.enableReport(SH2_ROTATION_VECTOR)) {
    std::cerr << "Could not enable rotation vector" << std::endl;
}

// Enable the accelerometer
if (!imu.enableReport(SH2_ACCELEROMETER, 5000)) { // 5ms interval
    std::cerr << "Could not enable accelerometer" << std::endl;
}
```

### 2. Poll for Sensor Events

After enabling the reports, you can get the sensor data by calling `getSensorEvent` in a loop. This function will return `true` when a new event is available.

The `sh2_SensorValue_t` struct contains the sensor data. You should use a `switch` statement on the `sensorId` field to determine which sensor the data is from, and then access the data through the `un` union.

```cpp
#include <iostream>
#include "BNO08x.h"

// ... initialize and enable reports ...

sh2_SensorValue_t sensorValue;

while (true) {
    if (imu.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ROTATION_VECTOR:
                std::cout << "Rotation Vector - R: " << sensorValue.un.rotationVector.real
                          << ", i: " << sensorValue.un.rotationVector.i
                          << ", j: " << sensorValue.un.rotationVector.j
                          << ", k: " << sensorValue.un.rotationVector.k << std::endl;
                break;
            case SH2_ACCELEROMETER:
                std::cout << "Accelerometer - x: " << sensorValue.un.accelerometer.x
                          << ", y: " << sensorValue.un.accelerometer.y
                          << ", z: " << sensorValue.un.accelerometer.z << std::endl;
                break;
        }
    }
}
```

### Available Sensors

Here is a list of some of the available sensors and their corresponding data structures in the `sh2_SensorValue_t` union:

| Sensor ID                       | Data Structure Member | Description                           |
| ------------------------------- | --------------------- | ------------------------------------- |
| `SH2_ACCELEROMETER`             | `accelerometer`       | Calibrated accelerometer data (m/s²)  |
| `SH2_GYROSCOPE_CALIBRATED`      | `gyroscope`           | Calibrated gyroscope data (rad/s)     |
| `SH2_MAGNETIC_FIELD_CALIBRATED` | `magneticField`       | Calibrated magnetometer data (µT)     |
| `SH2_ROTATION_VECTOR`           | `rotationVector`      | Fused rotation vector (quaternion)    |
| `SH2_LINEAR_ACCELERATION`       | `linearAcceleration`  | Linear acceleration (gravity removed) |
| `SH2_GRAVITY`                   | `gravity`             | Gravity vector                        |
| `SH2_TEMPERATURE`               | `temperature`         | Ambient temperature (°C)              |
| `SH2_PRESSURE`                  | `pressure`            | Atmospheric pressure (hPa)            |

For a complete list of sensors, please refer to the `sh2_SensorId_e` enum in `src/sh2.h`.

## License

This project is licensed under the terms of the MIT license. See the `license.txt` file for details.
