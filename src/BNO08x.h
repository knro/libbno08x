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
#include <stdexcept>
#include <string>

class BNO08x_exception : public std::runtime_error
{
    public:
        BNO08x_exception(const std::string &message) : std::runtime_error(message) {}
};

class BNO08x
{
        // Friend functions to allow C-style callbacks to access private members
        friend int hal_open_wrapper(sh2_Hal_t *self);
        friend void hal_close_wrapper(sh2_Hal_t *self);
        friend int hal_read_wrapper(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
        friend int hal_write_wrapper(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
        friend uint32_t hal_getTimeUs_wrapper(sh2_Hal_t *self);

    public:
        /**
         * @brief Construct a new BNO08x object
         * @param reset_pin The GPIO pin number for the reset pin. -1 if not used.
         */
        explicit BNO08x(int reset_pin = -1);

        /**
         * @brief Destroy the BNO08x object
         */
        ~BNO08x();

        /**
         * @brief Initialize the BNO08x with I2C communication.
         * @param i2c_addr The I2C address of the BNO08x.
         * @param i2c_bus The I2C bus device path.
         */
        void begin_i2c(uint8_t i2c_addr = 0x4A, const char* i2c_bus = "/dev/i2c-1");

        /**
         * @brief Initialize the BNO08x with SPI communication.
         * @param spi_device The SPI device path.
         * @param cs_pin The GPIO pin number for the chip select pin. -1 if not used.
         * @param int_pin The GPIO pin number for the interrupt pin. -1 if not used.
         */
        void begin_spi(const char* spi_device = "/dev/spidev0.0", int cs_pin = -1, int int_pin = -1);

        /**
         * @brief Perform a hardware reset of the BNO08x.
         */
        void hardwareReset();

        /**
         * @brief Check if a reset has occurred.
         * @return true if a reset has occurred, false otherwise.
         */
        [[nodiscard]] bool wasReset();

        /**
         * @brief Enable a sensor report.
         * @param sensor The sensor to enable.
         * @param interval_us The interval in microseconds between reports.
         * @return true if the report was enabled successfully, false otherwise.
         */
        [[nodiscard]] bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);

        /**
         * @brief Get a sensor event.
         * @param value A pointer to a sh2_SensorValue_t struct to store the event data.
         * @return true if a new event was received, false otherwise.
         */
        [[nodiscard]] bool getSensorEvent(sh2_SensorValue_t *value);

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
        comm_mode m_Mode;

        bool _init();
        sh2_Hal_t m_Hal;
        int m_ResetPin;
        int m_CsPin;
        int m_IntPin;
        bool m_ResetOccurred;
        sh2_SensorValue_t *m_SensorValue;
        uint8_t m_I2cAddr; // Store the I2C address

        // Advertisement data storage for proper initialization
        uint8_t m_InitialAdvertData[512];
        int m_InitialAdvertLen;
        bool m_InitialAdvertConsumed;
        int m_DevFd; // Device file descriptor (per instance)

        // HAL functions
        int hal_open();
        void hal_close();
        int hal_read(uint8_t *pBuffer, unsigned len, uint32_t *t_us);
        int hal_write(uint8_t *pBuffer, unsigned len);
        uint32_t hal_getTimeUs();

        static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
        static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);
};
