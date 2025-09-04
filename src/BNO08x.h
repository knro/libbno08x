/*
 * Copyright 2025, Jasem Mutlaq, Ikarus Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_hal.h"

class BNO08x
{
    public:
        /**
         * @brief Construct a new BNO08x object
         * @param reset_pin The GPIO pin number for the reset pin. -1 if not used.
         */
        BNO08x(int reset_pin = -1);

        /**
         * @brief Destroy the BNO08x object
         */
        ~BNO08x();

        /**
         * @brief Initialize the BNO08x with I2C communication.
         * @param i2c_addr The I2C address of the BNO08x.
         * @param i2c_bus The I2C bus device path.
         * @return true if initialization was successful, false otherwise.
         */
        bool begin_i2c(uint8_t i2c_addr = 0x4A, const char* i2c_bus = "/dev/i2c-1");

        /**
         * @brief Initialize the BNO08x with SPI communication.
         * @param spi_device The SPI device path.
         * @param cs_pin The GPIO pin number for the chip select pin. -1 if not used.
         * @param int_pin The GPIO pin number for the interrupt pin. -1 if not used.
         * @return true if initialization was successful, false otherwise.
         */
        bool begin_spi(const char* spi_device = "/dev/spidev0.0", int cs_pin = -1, int int_pin = -1);

        /**
         * @brief Perform a hardware reset of the BNO08x.
         */
        void hardwareReset();

        /**
         * @brief Check if a reset has occurred.
         * @return true if a reset has occurred, false otherwise.
         */
        bool wasReset();

        /**
         * @brief Enable a sensor report.
         * @param sensor The sensor to enable.
         * @param interval_us The interval in microseconds between reports.
         * @return true if the report was enabled successfully, false otherwise.
         */
        bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);

        /**
         * @brief Get a sensor event.
         * @param value A pointer to a sh2_SensorValue_t struct to store the event data.
         * @return true if a new event was received, false otherwise.
         */
        bool getSensorEvent(sh2_SensorValue_t *value);

        /**
         * @brief The product IDs of the BNO08x.
         */
        sh2_ProductIds_t prodIds;

    private:
        enum comm_mode
        {
            MODE_I2C,
            MODE_SPI
        };
        comm_mode _mode;

        bool _init();
        sh2_Hal_t _hal;
        int _reset_pin;
        int _cs_pin;
        int _int_pin;
        bool _reset_occurred;
        sh2_SensorValue_t *_sensor_value;

        // HAL function pointers
        static int hal_open(sh2_Hal_t *self);
        static void hal_close(sh2_Hal_t *self);
        static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
        static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
        static uint32_t hal_getTimeUs(sh2_Hal_t *self);

        static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
        static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);
};

