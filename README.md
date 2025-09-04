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

## License

This project is licensed under the terms of the MIT license. See the `license.txt` file for details.
