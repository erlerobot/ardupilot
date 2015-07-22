/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_AK8963.cpp 
 *       Code by Georgii Staroselskii. Emlid.com
 *
 *       Sensor is connected to SPI port
 *
 */

#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_AK8963.h"
#include "../AP_InertialSensor/AP_InertialSensor_MPU9250.h"

#define READ_FLAG                   0x80
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27
#define MPUREG_EXT_SENS_DATA_00     0x49
#define MPUREG_I2C_SLV0_DO          0x63

/* bit definitions for MPUREG_USER_CTRL */
#define MPUREG_USER_CTRL                                0x6A
/* Enable MPU to act as the I2C Master to external slave sensors */
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10

/* bit definitions for MPUREG_MST_CTRL */
#define MPUREG_I2C_MST_CTRL                             0x24
#        define I2C_SLV0_EN                             0x80
#        define I2C_MST_CLOCK_400KHZ                    0x0D
#        define I2C_MST_CLOCK_258KHZ                    0x08

#define AK8963_I2C_ADDR                                 0x0c

#define AK8963_WIA                                      0x00
#        define AK8963_Device_ID                        0x48

#define AK8963_INFO                                     0x01

#define AK8963_ST1                                      0x02
#        define AK8963_DRDY                             0x01
#        define AK8963_DOR                              0x02

#define AK8963_HXL                                      0x03

/* bit definitions for AK8963 CNTL1 */
#define AK8963_CNTL1                                    0x0A
#        define    AK8963_CONTINUOUS_MODE1              0x02
#        define    AK8963_CONTINUOUS_MODE2              0x06
#        define    AK8963_SELFTEST_MODE                 0x08
#        define    AK8963_POWERDOWN_MODE                0x00
#        define    AK8963_FUSE_MODE                     0x0f
#        define    AK8963_16BIT_ADC                     0x10
#        define    AK8963_14BIT_ADC                     0x00

#define AK8963_CNTL2                                    0x0B
#        define AK8963_RESET                            0x01

#define AK8963_ASAX                                     0x10

#define AK8963_DEBUG 0
#if AK8963_DEBUG
#include <stdio.h>
#define error(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ASSERT(x) assert(x)
#else
#define error(...) do { } while (0)
#ifndef ASSERT
#define ASSERT(x)
#endif
#endif

extern const AP_HAL::HAL& hal;

AP_Compass_AK8963::AP_Compass_AK8963(Compass &compass) :
    AP_Compass_Backend(compass),
    _initialized(false),
    _last_update_timestamp(0),
    _last_accum_time(0)
{
    _mag_x_accum =_mag_y_accum = _mag_z_accum = 0;
    _accum_count = 0;
}

AP_Compass_Backend *AP_Compass_AK8963::detect(Compass &compass)
{
    AP_Compass_AK8963 *sensor = new AP_Compass_AK8963(compass);

    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


/* stub to satisfy Compass API*/
void AP_Compass_AK8963::accumulate(void)
{
}

bool AP_Compass_AK8963::init()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU9250);

    if (_spi == NULL) {
        hal.console->println_P(PSTR("Cannot get SPIDevice_MPU9250"));
        return false;
    }

    _spi_sem = _spi->get_semaphore();

    hal.scheduler->suspend_timer_procs();

    if (!_spi_sem->take(100)) {
        hal.console->printf("AK8963: Unable to get MPU9250 semaphore");
        goto fail_sem;
    }

<<<<<<< HEAD
    if (!_configure_mpu9250()) {
        hal.console->printf_P(PSTR("AK8963: MPU9250 not configured for AK8963\n"));
=======
    if (!_bus->configure()) {
        hal.console->printf("AK8963: Could not configure bus for AK8963\n");
>>>>>>> e232543... AP_Compass: AK8963: change initialization and rename methods
        goto fail;
    }

    if (!_check_id()) {
        hal.console->printf("AK8963: Wrong id\n");
        goto fail;
    }

    if (!_calibrate()) {
        hal.console->printf("AK8963: Could not read calibration data\n");
        goto fail;
    }

    if (!_setup_mode()) {
        hal.console->printf("AK8963: Could not setup mode\n");
        goto fail;
    }

<<<<<<< HEAD
    if (!_start_conversion()) {
        hal.console->printf_P(PSTR("AK8963: conversion not started\n"));
=======
    if (!_bus->start_measurements()) {
        hal.console->printf("AK8963: Could not start measurments\n");
>>>>>>> e232543... AP_Compass: AK8963: change initialization and rename methods
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _compass_instance = register_compass();
<<<<<<< HEAD
    set_dev_id(_compass_instance, AP_COMPASS_TYPE_AK8963_MPU9250);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_AK8963::_update, void));

    _spi_sem->give();

=======
    set_dev_id(_compass_instance, _bus->get_dev_id());
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_AK8963::_update, void));

    _bus_sem->give();
>>>>>>> 27d95b6... AP_Compass: AK8963: remove state machine
    hal.scheduler->resume_timer_procs();

    return true;

fail:
    _spi_sem->give();
fail_sem:
    hal.scheduler->resume_timer_procs();

    return false;
}

void AP_Compass_AK8963::read()
{
    if (!_initialized) {
        return;
    }

    if (_accum_count == 0) {
        /* We're not ready to publish*/
        return;
    }

    /* Update */
    Vector3f field(_mag_x_accum * _magnetometer_ASA[0],
                   _mag_y_accum * _magnetometer_ASA[1],
                   _mag_z_accum * _magnetometer_ASA[2]);

    field /= _accum_count;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;
    _accum_count = 0;

<<<<<<< HEAD
=======
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    field.rotate(ROTATION_YAW_90);
#endif

>>>>>>> d941174... AP_Compass: AK8963: enhance the readability
    publish_field(field, _compass_instance);
}

void AP_Compass_AK8963::_update()
{
    if (hal.scheduler->micros() - _last_update_timestamp < 10000) {
        return;
    }

    if (!_sem_take_nonblocking()) {
        return;
    }

<<<<<<< HEAD
<<<<<<< HEAD
    switch (_state)
    {
        case STATE_SAMPLE:
            if (!_collect_samples()) {
                _state = STATE_ERROR;
            }
            break;
        case STATE_ERROR:
            if (_start_conversion()) {
                _state = STATE_SAMPLE;
            }
            break;
        default:
            break;
=======
    switch (_state) {
    case STATE_SAMPLE:
        if (!_collect_samples()) {
            _state = STATE_ERROR;
        }
        break;
    case STATE_ERROR:
        if (_bus->start_measurements()) {
            _state = STATE_SAMPLE;
        }
        break;
    default:
        break;
>>>>>>> d941174... AP_Compass: AK8963: enhance the readability
=======
    struct AP_AK8963_SerialBus::raw_value rv;

    _bus->read_raw(&rv);

    /* Check for overflow. See AK8963's datasheet, section
     * 6.4.3.6 - Magnetic Sensor Overflow. */
    if ((rv.st2 & 0x08)) {
        return;
    }

    float mag_x = (float) rv.val[0];
    float mag_y = (float) rv.val[1];
    float mag_z = (float) rv.val[2];

    if (is_zero(mag_x) && is_zero(mag_y) && is_zero(mag_z)) {
        return;
    }

    _mag_x_accum += mag_x;
    _mag_y_accum += mag_y;
    _mag_z_accum += mag_z;
    _accum_count++;
    if (_accum_count == 10) {
        _mag_x_accum /= 2;
        _mag_y_accum /= 2;
        _mag_z_accum /= 2;
        _accum_count = 5;
>>>>>>> 27d95b6... AP_Compass: AK8963: remove state machine
    }

    _last_update_timestamp = hal.scheduler->micros();
    _sem_give();
}

bool AP_Compass_AK8963::_check_id()
{
    for (int i = 0; i < 5; i++) {
<<<<<<< HEAD
        uint8_t deviceid;
        _register_read(AK8963_WIA, &deviceid, 0x01); /* Read AK8963's id */
=======
        uint8_t deviceid = 0;
        _bus->register_read(AK8963_WIA, &deviceid, 0x01); /* Read AK8963's id */
>>>>>>> d941174... AP_Compass: AK8963: enhance the readability

        if (deviceid == AK8963_Device_ID) {
            return true;
        }
    }

    return false;
}

<<<<<<< HEAD
bool AP_Compass_AK8963::_configure_mpu9250()
{
    if (!AP_InertialSensor_MPU9250::initialize_driver_state())
        return false;

    uint8_t user_ctrl;
    _register_read(MPUREG_USER_CTRL, &user_ctrl, 1);
    _bus_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_I2C_MST_EN);
    _bus_write(MPUREG_I2C_MST_CTRL, I2C_MST_CLOCK_400KHZ);

    return true;
}

bool AP_Compass_AK8963::_configure() {
<<<<<<< HEAD
    _register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | _magnetometer_adc_resolution);
=======
=======
bool AP_Compass_AK8963::_setup_mode() {
>>>>>>> e232543... AP_Compass: AK8963: change initialization and rename methods
    _bus->register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);
>>>>>>> 28d3d77... AP_Compass: AK8963: remove resolution member
    return true;
}

bool AP_Compass_AK8963::_reset()
{
    _register_write(AK8963_CNTL2, AK8963_RESET);

    return true;
}


bool AP_Compass_AK8963::_calibrate()
{
<<<<<<< HEAD
    uint8_t cntl1 = _register_read(AK8963_CNTL1);

<<<<<<< HEAD
    _register_write(AK8963_CNTL1, AK8963_FUSE_MODE | _magnetometer_adc_resolution); /* Enable FUSE-mode in order to be able to read calibreation data */
=======
=======
>>>>>>> e232543... AP_Compass: AK8963: change initialization and rename methods
    /* Enable FUSE-mode in order to be able to read calibration data */
<<<<<<< HEAD
    _bus->register_write(AK8963_CNTL1, AK8963_FUSE_MODE | _magnetometer_adc_resolution);
>>>>>>> d941174... AP_Compass: AK8963: enhance the readability
=======
    _bus->register_write(AK8963_CNTL1, AK8963_FUSE_MODE | AK8963_16BIT_ADC);
>>>>>>> 28d3d77... AP_Compass: AK8963: remove resolution member

    uint8_t response[3];
    _register_read(AK8963_ASAX, response, 3);

    for (int i = 0; i < 3; i++) {
        float data = response[i];
        _magnetometer_ASA[i] = ((data - 128) / 256 + 1);
        error("%d: %lf\n", i, _magnetometer_ASA[i]);
    }

<<<<<<< HEAD
    _register_write(AK8963_CNTL1, cntl1);

    return true;
}

bool AP_Compass_AK8963::_start_conversion()
{
    static const uint8_t address = AK8963_INFO;
    /* Read registers from INFO through ST2 */
    static const uint8_t count = 0x09;

    _configure_mpu9250();
    _bus_write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  /* Set the I2C slave addres of AK8963 and set for read. */
    _bus_write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _bus_write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count); /* Enable I2C and set @count byte */

=======
>>>>>>> e232543... AP_Compass: AK8963: change initialization and rename methods
    return true;
}

<<<<<<< HEAD
bool AP_Compass_AK8963::_collect_samples()
{
    struct AP_AK8963_SerialBus::raw_value rv;

    if (!_initialized) {
        return false;
    }

<<<<<<< HEAD
    if (!_read_raw()) {
=======
    _bus->read_raw(&rv);
    if ((rv.st2 & 0x08)) {
>>>>>>> 86b3312... AP_Compass: AK8963: factor out common code of read_raw()
        return false;
    }

    float mag_x = (float) rv.val[0];
    float mag_y = (float) rv.val[1];
    float mag_z = (float) rv.val[2];

    if (is_zero(mag_x) && is_zero(mag_y) && is_zero(mag_z)) {
        return false;
    }

    _mag_x_accum += mag_x;
    _mag_y_accum += mag_y;
    _mag_z_accum += mag_z;
    _accum_count++;
    if (_accum_count == 10) {
        _mag_x_accum /= 2;
        _mag_y_accum /= 2;
        _mag_z_accum /= 2;
        _accum_count = 5;
    }

    return true;
}

=======
>>>>>>> 27d95b6... AP_Compass: AK8963: remove state machine
bool AP_Compass_AK8963::_sem_take_blocking()
{
    return _spi_sem->take(10);
}

bool AP_Compass_AK8963::_sem_give()
{
    return _spi_sem->give();
}

bool AP_Compass_AK8963::_sem_take_nonblocking()
{
    static int _sem_failure_count = 0;

<<<<<<< HEAD
    bool got = _spi_sem->take_nonblocking();

    if (!got) {
        if (!hal.scheduler->system_initializing()) {
            _sem_failure_count++;
            if (_sem_failure_count > 100) {
                hal.scheduler->panic(PSTR("PANIC: failed to take _spi_sem "
                                          "100 times in a row, in "
                                          "AP_Compass_AK8963::_update"));
            }
=======
    if (_bus_sem->take_nonblocking()) {
        _sem_failure_count = 0;
        return true;
    }

    if (!hal.scheduler->system_initializing() ) {
        _sem_failure_count++;
        if (_sem_failure_count > 100) {
<<<<<<< HEAD
            hal.scheduler->panic(PSTR("PANIC: failed to take _bus->sem "
                                      "100 times in a row, in "
                                      "AP_Compass_AK8963"));
>>>>>>> d941174... AP_Compass: AK8963: enhance the readability
=======
            hal.scheduler->panic("PANIC: failed to take _bus->sem "
                                 "100 times in a row, in "
                                 "AP_Compass_AK8963");
>>>>>>> e232543... AP_Compass: AK8963: change initialization and rename methods
        }
    }

    return false;
}

void AP_Compass_AK8963::_dump_registers()
{
#if AK8963_DEBUG
    error("MPU9250 registers\n");
    static uint8_t regs[0x7e];

    _bus_read(0x0, regs, 0x7e);

    for (uint8_t reg=0x00; reg<=0x7E; reg++) {
        uint8_t v = regs[reg];
        error(("%d:%02x "), (unsigned)reg, (unsigned)v);
        if (reg  % 16 == 0) {
            error("\n");
        }
    }
    error("\n");
#endif
}

<<<<<<< HEAD
bool AP_Compass_AK8963::_read_raw()
{
    uint8_t rx[14] = {0};

    const uint8_t count = 9;
    _bus_read(MPUREG_EXT_SENS_DATA_00, rx, count);

    uint8_t st2 = rx[8]; /* End data read by reading ST2 register */

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx + 1] << 8) | v[2*idx]))

<<<<<<< HEAD
    if(!(st2 & 0x08)) {
        _mag_x = (float) int16_val(rx, 1);
        _mag_y = (float) int16_val(rx, 2);
        _mag_z = (float) int16_val(rx, 3);

        if (is_zero(_mag_x) && is_zero(_mag_y) && is_zero(_mag_z)) {
            return false;
        }
=======
    if (st2 & 0x08) {
        return false;
    }

    mag_x = (float) int16_val(rx, 1);
    mag_y = (float) int16_val(rx, 2);
    mag_z = (float) int16_val(rx, 3);
>>>>>>> d941174... AP_Compass: AK8963: enhance the readability

    if (is_zero(mag_x) && is_zero(mag_y) && is_zero(mag_z)) {
        return false;
    }
<<<<<<< HEAD
=======

    return true;
=======
/* MPU9250 implementation of the AK8963 */
AP_AK8963_SerialBus_MPU9250::AP_AK8963_SerialBus_MPU9250()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU9250);

    if (_spi == NULL) {
        hal.console->printf("Cannot get SPIDevice_MPU9250\n");
        return;
    }
}

void AP_AK8963_SerialBus_MPU9250::register_write(uint8_t address, uint8_t value)
{
    const uint8_t count = 1;
    _write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    _write(MPUREG_I2C_SLV0_REG, address);
    _write(MPUREG_I2C_SLV0_DO, value);
    _write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count);
}

void AP_AK8963_SerialBus_MPU9250::register_read(uint8_t address, uint8_t *value, uint8_t count)
{
    _write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);
    _write(MPUREG_I2C_SLV0_REG, address);
    _write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count);

    hal.scheduler->delay(10);
    _read(MPUREG_EXT_SENS_DATA_00, value, count);
}

void AP_AK8963_SerialBus_MPU9250::_read(uint8_t address, uint8_t *buf, uint32_t count)
{
    ASSERT(count < 150);
    uint8_t tx[150];
    uint8_t rx[150];

    tx[0] = address | READ_FLAG;
    tx[1] = 0;
    _spi->transaction(tx, rx, count + 1);

    memcpy(buf, rx + 1, count);
}

void AP_AK8963_SerialBus_MPU9250::_write(uint8_t address, const uint8_t *buf, uint32_t count)
{
    ASSERT(count < 2);
    uint8_t tx[2];

    tx[0] = address;
    memcpy(tx+1, buf, count);

    _spi->transaction(tx, NULL, count + 1);
}

bool AP_AK8963_SerialBus_MPU9250::configure()
{
    if (!AP_InertialSensor_MPU9250::initialize_driver_state())
        return false;

    uint8_t user_ctrl;
    register_read(MPUREG_USER_CTRL, &user_ctrl, 1);
    _write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_I2C_MST_EN);
    _write(MPUREG_I2C_MST_CTRL, I2C_MST_CLOCK_400KHZ);

    return true;
}

void AP_AK8963_SerialBus_MPU9250::read_raw(struct raw_value *rv)
{
    _read(MPUREG_EXT_SENS_DATA_00, (uint8_t *) rv, sizeof(*rv));
>>>>>>> 86b3312... AP_Compass: AK8963: factor out common code of read_raw()
}

AP_HAL::Semaphore * AP_AK8963_SerialBus_MPU9250::get_semaphore()
{
    return _spi->get_semaphore();
}

bool AP_AK8963_SerialBus_MPU9250::start_measurements()
{
    const uint8_t count = sizeof(struct raw_value);

    /* Configure the registers from AK8963 that will be read by MPU9250's
     * master: we will get the result directly from MPU9250's registers starting
     * from MPUREG_EXT_SENS_DATA_00 when read_raw() is called */
    _write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);
    _write(MPUREG_I2C_SLV0_REG, AK8963_INFO);
    _write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count);

    return true;
}
>>>>>>> d941174... AP_Compass: AK8963: enhance the readability

}
void AP_Compass_AK8963::_register_write(uint8_t address, uint8_t value)
{
    _bus_write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);  /* Set the I2C slave addres of AK8963 and set for _register_write. */
    _bus_write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _bus_write(MPUREG_I2C_SLV0_DO, value); /* Register value to continuous measurement in 16-bit */
    _bus_write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | 0x01); /* Enable I2C and set 1 byte */
}

void AP_Compass_AK8963::_register_read(uint8_t address, uint8_t *value, uint8_t count)
{
    _bus_write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  /* Set the I2C slave addres of AK8963 and set for read. */
    _bus_write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _bus_write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count); /* Enable I2C and set @count byte */

    hal.scheduler->delay(10);
    _bus_read(MPUREG_EXT_SENS_DATA_00, value, count);
}

<<<<<<< HEAD
void AP_Compass_AK8963::_bus_read(uint8_t address, uint8_t *buf, uint32_t count)
{
<<<<<<< HEAD
    ASSERT(count < 150);
    uint8_t tx[150];
    uint8_t rx[150];

    tx[0] = address | READ_FLAG;
    tx[1] = 0;
    _spi->transaction(tx, rx, count + 1);

    memcpy(buf, rx + 1, count);
=======
    uint8_t rx[9] = {0};

    const uint8_t count = 9;
    _i2c->readRegisters(_addr, AK8963_INFO, count, rx);

    uint8_t st2 = rx[8]; /* End data read by reading ST2 register */

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx + 1] << 8) | v[2*idx]))

    if (st2 & 0x08) {
        return false;
    }

    mag_x = (float) int16_val(rx, 1);
    mag_y = (float) int16_val(rx, 2);
    mag_z = (float) int16_val(rx, 3);

    if (is_zero(mag_x) && is_zero(mag_y) && is_zero(mag_z)) {
        return false;
    }

    return true;
>>>>>>> d941174... AP_Compass: AK8963: enhance the readability
=======
void AP_AK8963_SerialBus_I2C::read_raw(struct raw_value *rv)
{
    _i2c->readRegisters(_addr, AK8963_INFO, sizeof(*rv), (uint8_t *) rv);
>>>>>>> 86b3312... AP_Compass: AK8963: factor out common code of read_raw()
}

void AP_Compass_AK8963::_bus_write(uint8_t address, const uint8_t *buf, uint32_t count)
{
    ASSERT(count < 2);
    uint8_t tx[2];

    tx[0] = address;
    memcpy(tx+1, buf, count);

    _spi->transaction(tx, NULL, count + 1);
}
