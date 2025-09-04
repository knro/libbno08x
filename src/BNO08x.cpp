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
#include <iostream>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#ifdef HAVE_LIBGPIOD_V2
#include <gpiod.hpp>
#else
#include <gpiod.h>
#endif

// Placeholder for I2C/SPI device handle
static int dev_fd = -1;

BNO08x::BNO08x(int reset_pin) : _reset_pin(reset_pin), _reset_occurred(false)
{
}

BNO08x::~BNO08x()
{
    if (dev_fd != -1)
    {
        hal_close(&_hal);
    }
}

bool BNO08x::begin_i2c(uint8_t i2c_addr, const char* i2c_bus)
{
    _mode = MODE_I2C;
    _hal.open = hal_open;
    _hal.close = hal_close;
    _hal.read = hal_read;
    _hal.write = hal_write;
    _hal.getTimeUs = hal_getTimeUs;

    dev_fd = open(i2c_bus, O_RDWR);
    if (dev_fd < 0)
    {
        return false;
    }

    if (ioctl(dev_fd, I2C_SLAVE, i2c_addr) < 0)
    {
        close(dev_fd);
        dev_fd = -1;
        return false;
    }

    return _init();
}

bool BNO08x::begin_spi(const char* spi_device, int cs_pin, int int_pin)
{
    _mode = MODE_SPI;
    _cs_pin = cs_pin;
    _int_pin = int_pin;

    _hal.open = hal_open;
    _hal.close = hal_close;
    _hal.read = hal_read;
    _hal.write = hal_write;
    _hal.getTimeUs = hal_getTimeUs;

    dev_fd = open(spi_device, O_RDWR);
    if (dev_fd < 0)
    {
        return false;
    }

    uint8_t mode = SPI_MODE_3;
    if (ioctl(dev_fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        close(dev_fd);
        dev_fd = -1;
        return false;
    }

    uint8_t bpw = 8;
    if (ioctl(dev_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0)
    {
        close(dev_fd);
        dev_fd = -1;
        return false;
    }

    uint32_t speed = 3000000;
    if (ioctl(dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        close(dev_fd);
        dev_fd = -1;
        return false;
    }

    return _init();
}

void BNO08x::hardwareReset()
{
    if (_reset_pin == -1)
    {
        return;
    }

#ifdef HAVE_LIBGPIOD_V2
    try
    {
        gpiod::chip chip("/dev/gpiochip0");
        auto request = chip.prepare_request()
                       .set_consumer("BNO08x")
                       .add_line_settings(_reset_pin,
                                          gpiod::line_settings()
                                          .set_direction(gpiod::line::direction::OUTPUT)
                                          .set_output_value(gpiod::line::value::ACTIVE))
                       .do_request();

        usleep(10000);
        request.set_value(_reset_pin, gpiod::line::value::INACTIVE);
        usleep(10000);
        request.set_value(_reset_pin, gpiod::line::value::ACTIVE);
        usleep(10000);
        request.release();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to reset BNO08x via GPIO: " << e.what() << std::endl;
    }
#else
    struct gpiod_chip *chip;
    struct gpiod_line *line;

    chip = gpiod_chip_open_by_number(0);
    if (!chip)
    {
        return;
    }

    line = gpiod_chip_get_line(chip, _reset_pin);
    if (!line)
    {
        gpiod_chip_close(chip);
        return;
    }

    gpiod_line_request_output(line, "bno08x-reset", 1);
    usleep(10000);
    gpiod_line_set_value(line, 0);
    usleep(10000);
    gpiod_line_set_value(line, 1);
    usleep(10000);

    gpiod_line_release(line);
    gpiod_chip_close(chip);
#endif
}

bool BNO08x::wasReset()
{
    bool r = _reset_occurred;
    _reset_occurred = false;
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
    _sensor_value = value;
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
    status = sh2_open(&_hal, hal_callback, this);
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

int BNO08x::hal_open(sh2_Hal_t *self)
{
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    int ret = write(dev_fd, softreset_pkt, 5);
    if (ret != 5)
    {
        return -1;
    }
    usleep(300000);
    return 0;
}

void BNO08x::hal_close(sh2_Hal_t *self)
{
    if (dev_fd != -1)
    {
        close(dev_fd);
        dev_fd = -1;
    }
}

int BNO08x::hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    BNO08x *bno08x = (BNO08x *)self;
    if (bno08x->_mode == MODE_I2C)
    {
        uint8_t header[4];
        if (read(dev_fd, header, 4) != 4)
        {
            return 0;
        }

        uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
        packet_size &= ~0x8000;

        if (packet_size > len)
        {
            return 0;
        }

        if (read(dev_fd, pBuffer, packet_size) != packet_size)
        {
            return 0;
        }

        *t_us = hal_getTimeUs(self);

        return packet_size;
    }
    else     // MODE_SPI
    {
        struct spi_ioc_transfer xfer;
        memset(&xfer, 0, sizeof(xfer));

        uint8_t header[4];
        xfer.tx_buf = (unsigned long)NULL;
        xfer.rx_buf = (unsigned long)header;
        xfer.len = 4;
        if (ioctl(dev_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
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
        if (ioctl(dev_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
        {
            return 0;
        }

        *t_us = hal_getTimeUs(self);

        return packet_size;
    }
}

int BNO08x::hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    BNO08x *bno08x = (BNO08x *)self;
    if (bno08x->_mode == MODE_I2C)
    {
        int ret = write(dev_fd, pBuffer, len);
        if (ret != len)
        {
            return 0;
        }
        return ret;
    }
    else     // MODE_SPI
    {
        struct spi_ioc_transfer xfer;
        memset(&xfer, 0, sizeof(xfer));

        xfer.tx_buf = (unsigned long)pBuffer;
        xfer.rx_buf = (unsigned long)NULL;
        xfer.len = len;
        if (ioctl(dev_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
        {
            return 0;
        }
        return len;
    }
}

uint32_t BNO08x::hal_getTimeUs(sh2_Hal_t *self)
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
        self->_reset_occurred = true;
    }
}

// Handle sensor events.
void BNO08x::sensorHandler(void *cookie, sh2_SensorEvent_t *event)
{
    BNO08x *self = (BNO08x *)cookie;
    int rc;

    rc = sh2_decodeSensorEvent(self->_sensor_value, event);
    if (rc != SH2_OK)
    {
        // Error decoding sensor event
        self->_sensor_value->timestamp = 0;
        return;
    }
}
