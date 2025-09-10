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

#include "BNO08x.h"
#include "sh2_err.h"
#include "config.h"
#include <iostream>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <gpiod.hpp>
#include <cstdio>
#include <cerrno>
#include <linux/i2c.h>

BNO08x::BNO08x(int reset_pin) : m_ResetPin(reset_pin), m_ResetOccurred(false),
    m_InitialAdvertLen(0), m_InitialAdvertConsumed(false), m_DevFd(-1)
{
}

BNO08x::~BNO08x()
{
    if (m_DevFd != -1)
    {
        hal_close();
    }
}

// Wrapper functions to provide a C-style interface for the sh2 library
int hal_open_wrapper(sh2_Hal_t *self)
{
    return static_cast<BNO08x*>(self->self)->hal_open();
}

void hal_close_wrapper(sh2_Hal_t *self)
{
    static_cast<BNO08x*>(self->self)->hal_close();
}

int hal_read_wrapper(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    return static_cast<BNO08x*>(self->self)->hal_read(pBuffer, len, t_us);
}

int hal_write_wrapper(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    return static_cast<BNO08x*>(self->self)->hal_write(pBuffer, len);
}

uint32_t hal_getTimeUs_wrapper(sh2_Hal_t *self)
{
    return static_cast<BNO08x*>(self->self)->hal_getTimeUs();
}

void BNO08x::begin_i2c(uint8_t i2c_addr, const char* i2c_bus)
{
    m_Mode = MODE_I2C;
    m_I2cAddr = i2c_addr;
    m_Hal.open = hal_open_wrapper;
    m_Hal.close = hal_close_wrapper;
    m_Hal.read = hal_read_wrapper;
    m_Hal.write = hal_write_wrapper;
    m_Hal.getTimeUs = hal_getTimeUs_wrapper;
    m_Hal.self = this;

    m_DevFd = open(i2c_bus, O_RDWR);
    if (m_DevFd < 0)
    {
        throw BNO08x_exception("Failed to open I2C bus: " + std::string(i2c_bus));
    }

    if (ioctl(m_DevFd, I2C_SLAVE, i2c_addr) < 0)
    {
        close(m_DevFd);
        m_DevFd = -1;
        throw BNO08x_exception("Failed to set I2C slave address: " + std::to_string(i2c_addr));
    }

    if (!_init())
    {
        throw BNO08x_exception("Failed to initialize BNO08x.");
    }
}

void BNO08x::begin_i2c(int devFd)
{
    m_Mode = MODE_I2C;
    m_DevFd = devFd;
    m_Hal.open = hal_open_wrapper;
    m_Hal.close = hal_close_wrapper;
    m_Hal.read = hal_read_wrapper;
    m_Hal.write = hal_write_wrapper;
    m_Hal.getTimeUs = hal_getTimeUs_wrapper;
    m_Hal.self = this;

    if (!_init())
    {
        throw BNO08x_exception("Failed to initialize BNO08x with external file descriptor.");
    }
}

void BNO08x::begin_spi(const char* spi_device, int cs_pin, int int_pin)
{
    m_Mode = MODE_SPI;
    m_CsPin = cs_pin;
    m_IntPin = int_pin;

    m_Hal.open = hal_open_wrapper;
    m_Hal.close = hal_close_wrapper;
    m_Hal.read = hal_read_wrapper;
    m_Hal.write = hal_write_wrapper;
    m_Hal.getTimeUs = hal_getTimeUs_wrapper;
    m_Hal.self = this;

    m_DevFd = open(spi_device, O_RDWR);
    if (m_DevFd < 0)
    {
        throw BNO08x_exception("Failed to open SPI device: " + std::string(spi_device));
    }

    uint8_t mode = SPI_MODE_3;
    if (ioctl(m_DevFd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        close(m_DevFd);
        m_DevFd = -1;
        throw BNO08x_exception("Failed to set SPI mode.");
    }

    uint8_t bpw = 8;
    if (ioctl(m_DevFd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0)
    {
        close(m_DevFd);
        m_DevFd = -1;
        throw BNO08x_exception("Failed to set SPI bits per word.");
    }

    uint32_t speed = 3000000;
    if (ioctl(m_DevFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        close(m_DevFd);
        m_DevFd = -1;
        throw BNO08x_exception("Failed to set SPI max speed.");
    }

    if (!_init())
    {
        throw BNO08x_exception("Failed to initialize BNO08x.");
    }
}

namespace
{
void gpio_set_value(int pin, int value)
{
#ifdef HAVE_LIBGPIOD_V2
    gpiod::chip chip("/dev/gpiochip0");
    auto request = chip.prepare_request()
                   .set_consumer("BNO08x")
                   .add_line_settings(pin,
                                      gpiod::line_settings()
                                      .set_direction(gpiod::line::direction::OUTPUT)
                                      .set_output_value(static_cast<gpiod::line::value>(value)))
                   .do_request();
    usleep(10000);
    request.release();
#else
    gpiod::chip chip("/dev/gpiochip0", gpiod::chip::OPEN_BY_PATH);
    gpiod::line line = chip.get_line(pin);
    line.request({ "bno08x-reset", gpiod::line_request::DIRECTION_OUTPUT }, value);
    usleep(10000);
    line.release();
#endif
}
}

void BNO08x::hardwareReset()
{
    if (m_ResetPin == -1)
    {
        return;
    }

    try
    {
        gpio_set_value(m_ResetPin, 1);
        gpio_set_value(m_ResetPin, 0);
        gpio_set_value(m_ResetPin, 1);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to reset BNO08x via GPIO: " << e.what() << std::endl;
    }
}

bool BNO08x::wasReset()
{
    bool r = m_ResetOccurred;
    m_ResetOccurred = false;
    return r;
}

bool BNO08x::enableReport(sh2_SensorId_t sensor, uint32_t interval_us)
{
    sh2_SensorConfig_t config;

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    config.reportInterval_us = interval_us;
    int status = sh2_setSensorConfig(sensor, &config);

    if (status != SH2_OK)
    {
        return false;
    }

    return true;
}

bool BNO08x::getSensorEvent(sh2_SensorValue_t *value)
{
    m_SensorValue = value;
    value->timestamp = 0;

    sh2_service();

    if (value->timestamp == 0)
    {
        // no new events
        return false;
    }

    return true;
}

bool BNO08x::_init()
{
    int status;

    hardwareReset();

    // Open SH2 interface (also registers non-sensor event handler.)
    status = sh2_open(&m_Hal, hal_callback, this);
    if (status != SH2_OK)
    {
        return false;
    }

    // Check connection partially by getting the product id's
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK)
    {
        return false;
    }

    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, this);

    return true;
}

int BNO08x::hal_open()
{
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    bool success = false;

    // Reset the initial advertisement data state
    m_InitialAdvertLen = 0;
    m_InitialAdvertConsumed = false;

    // Implement Adafruit-style retry mechanism for initialization only
    for (uint8_t attempts = 0; attempts < 5; attempts++)
    {
        int ret = write(m_DevFd, softreset_pkt, 5);
        if (ret == 5)
        {
            success = true;
            break;
        }
        usleep(30000); // 30ms delay between attempts (matching Adafruit approach)
    }

    if (!success)
    {
        return -1;
    }

    // Wait for sensor to be ready and send advertisement data
    usleep(500000); // 500ms delay to match our successful test

    // Now immediately read the advertisement data (like our successful enhanced test)
    m_InitialAdvertLen = read(m_DevFd, m_InitialAdvertData, sizeof(m_InitialAdvertData));

    return 0;
}

void BNO08x::hal_close()
{
    if (m_DevFd != -1)
    {
        close(m_DevFd);
        m_DevFd = -1;
    }
}

int BNO08x::hal_read(uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    if (m_Mode == MODE_I2C)
    {
        if (!m_InitialAdvertConsumed && m_InitialAdvertLen > 0)
        {
            size_t copy_len = (static_cast<size_t>(m_InitialAdvertLen) < len) ? static_cast<size_t>(m_InitialAdvertLen) : len;
            memcpy(pBuffer, m_InitialAdvertData, copy_len);
            m_InitialAdvertConsumed = true;
            *t_us = hal_getTimeUs();
            return copy_len;
        }

        ssize_t result = read(m_DevFd, pBuffer, len);
        if (result > 0)
        {
            *t_us = hal_getTimeUs();
            return result;
        }
        else
        {
            return 0;
        }
    }
    else     // MODE_SPI
    {
        struct spi_ioc_transfer xfer;
        memset(&xfer, 0, sizeof(xfer));

        uint8_t header[4];
        xfer.tx_buf = (unsigned long)NULL;
        xfer.rx_buf = (unsigned long)header;
        xfer.len = 4;
        if (ioctl(m_DevFd, SPI_IOC_MESSAGE(1), &xfer) < 0)
        {
            return 0;
        }

        uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
        packet_size &= ~0x8000;

        if (packet_size > len)
        {
            return 0;
        }

        xfer.tx_buf = (unsigned long)NULL;
        xfer.rx_buf = (unsigned long)pBuffer;
        xfer.len = packet_size;
        if (ioctl(m_DevFd, SPI_IOC_MESSAGE(1), &xfer) < 0)
        {
            return 0;
        }

        *t_us = hal_getTimeUs();

        return packet_size;
    }
}

int BNO08x::hal_write(uint8_t *pBuffer, unsigned len)
{
    if (m_Mode == MODE_I2C)
    {
        ssize_t ret = write(m_DevFd, pBuffer, len);
        if (ret < 0 || static_cast<unsigned>(ret) != len)
        {
            return 0;
        }
        usleep(1000); // Add a small delay after write
        return ret;
    }
    else     // MODE_SPI
    {
        struct spi_ioc_transfer xfer;
        memset(&xfer, 0, sizeof(xfer));

        xfer.tx_buf = (unsigned long)pBuffer;
        xfer.rx_buf = (unsigned long)NULL;
        xfer.len = len;
        if (ioctl(m_DevFd, SPI_IOC_MESSAGE(1), &xfer) < 0)
        {
            return 0;
        }
        return len;
    }
}

uint32_t BNO08x::hal_getTimeUs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000000) + (ts.tv_nsec / 1000);
}

void BNO08x::hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent)
{
    BNO08x *self = (BNO08x *)cookie;
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET)
    {
        self->m_ResetOccurred = true;
    }
}

// Handle sensor events.
void BNO08x::sensorHandler(void *cookie, sh2_SensorEvent_t *event)
{
    BNO08x *self = (BNO08x *)cookie;
    int rc;

    rc = sh2_decodeSensorEvent(self->m_SensorValue, event);
    if (rc != SH2_OK)
    {
        // Error decoding sensor event
        self->m_SensorValue->timestamp = 0;
        return;
    }
}
