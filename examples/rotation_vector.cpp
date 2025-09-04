#include <iostream>
#include <iomanip>
#include "BNO08x.h"

int main()
{
    BNO08x bno08x;
    sh2_SensorValue_t sensorValue;

    if (!bno08x.begin_i2c())
    {
        std::cerr << "Failed to find BNO08x chip" << std::endl;
        return -1;
    }

    std::cout << "BNO08x found!" << std::endl;

    if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
    {
        std::cerr << "Could not enable rotation vector" << std::endl;
        return -1;
    }

    while (true)
    {
        if (bno08x.getSensorEvent(&sensorValue))
        {
            switch (sensorValue.sensorId)
            {
                case SH2_ROTATION_VECTOR:
                    std::cout << "Rotation Vector - R: " << std::fixed << std::setprecision(4)
                              << sensorValue.un.rotationVector.real
                              << ", i: " << sensorValue.un.rotationVector.i
                              << ", j: " << sensorValue.un.rotationVector.j
                              << ", k: " << sensorValue.un.rotationVector.k << std::endl;
                    break;
            }
        }
    }

    return 0;
}
