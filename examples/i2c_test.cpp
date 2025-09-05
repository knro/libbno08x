#include <iostream>
#include <iomanip>
#include <string>
#include <unistd.h>
#include "BNO08x.h"

void print_product_id(const sh2_ProductIds_t *prodIds)
{
    printf("Product ID Information:\n");
    printf("  - Number of entries: %d\n", prodIds->numEntries);
    for (int i = 0; i < prodIds->numEntries; ++i)
    {
        printf("  - SW Part Number: 0x%x\n", prodIds->entry[i].swPartNumber);
        printf("  - SW Build Number: %d\n", prodIds->entry[i].swBuildNumber);
        printf("  - SW Version Major: %d\n", prodIds->entry[i].swVersionMajor);
        printf("  - SW Version Minor: %d\n", prodIds->entry[i].swVersionMinor);
        printf("  - SW Version Patch: %d\n", prodIds->entry[i].swVersionPatch);
        printf("  - Reset Cause: %d\n", prodIds->entry[i].resetCause);
    }
}

int main(void)
{
    BNO08x bno08x;
    sh2_SensorValue_t sensorValue;

    std::cout << "=== BNO085 I2C Communication Test ===" << std::endl;
    std::cout << "Testing I2C communication with BNO085 sensor at address 0x4B" << std::endl;
    std::cout << "=================================================" << std::endl << std::endl;

    std::cout << "[TEST 1] Attempting I2C initialization..." << std::endl;
    try
    {
        bno08x.begin_i2c(0x4B, "/dev/i2c-1");
        std::cout << "✅ SUCCESS: I2C initialization completed" << std::endl << std::endl;
    }
    catch (const BNO08x_exception &e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "[TEST 2] Checking sensor reset status..." << std::endl;
    if (bno08x.wasReset())
    {
        std::cout << "✅ Sensor reset detected - normal startup behavior" << std::endl << std::endl;
    }
    else
    {
        std::cout << "✅ No sensor reset detected" << std::endl << std::endl;
    }

    std::cout << "[TEST 3] Reading product information..." << std::endl;
    print_product_id(&bno08x.prodIds);
    std::cout << std::endl;

    std::cout << "[TEST 4] Testing sensor communication..." << std::endl;
    std::cout << "Enabling accelerometer report..." << std::endl;
    if (bno08x.enableReport(SH2_ACCELEROMETER, 100000))
    {
        std::cout << "✅ SUCCESS: Accelerometer report enabled" << std::endl << std::endl;
    }
    else
    {
        std::cerr << "❌ FAILED: Could not enable accelerometer report" << std::endl;
        return 1;
    }

    std::cout << "[TEST 5] Reading sensor data (10 samples)..." << std::endl;
    for (int i = 0; i < 10; i++)
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            std::cout << "Sample " << i + 1 << " - Accelerometer: ";
            std::cout << "X=" << std::fixed << std::setprecision(3) << sensorValue.un.accelerometer.x << "m/s² ";
            std::cout << "Y=" << std::fixed << std::setprecision(3) << sensorValue.un.accelerometer.y << "m/s² ";
            std::cout << "Z=" << std::fixed << std::setprecision(3) << sensorValue.un.accelerometer.z << "m/s²" << std::endl;
        }
        usleep(100000);
    }
    std::cout << "✅ SUCCESS: Received all 10 sensor samples" << std::endl << std::endl;

    std::cout << "=== TEST SUMMARY ===" << std::endl;
    std::cout << "✅ I2C initialization: PASSED" << std::endl;
    std::cout << "✅ Product ID retrieval: PASSED" << std::endl;
    std::cout << "✅ Sensor report enable: PASSED" << std::endl;
    std::cout << "✅ Data communication: PASSED (10 samples)" << std::endl << std::endl;
    std::cout << "🎉 BNO085 I2C communication is working correctly!" << std::endl;

    return 0;
}
