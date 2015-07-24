/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_AK8963_H
#define AP_Compass_AK8963_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"
#include "AP_Compass_Backend.h"

<<<<<<< HEAD
=======
class AP_AK8963_SerialBus
{
public:
    struct PACKED raw_value {
        uint8_t info;
        uint8_t st1;
        int16_t val[3];
        uint8_t st2;
    };

    virtual void register_read(uint8_t address, uint8_t *value, uint8_t count) = 0;
    uint8_t register_read(uint8_t address) {
        uint8_t reg;
        register_read(address, &reg, 1);
        return reg;
    }
    virtual void register_write(uint8_t address, uint8_t value) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual bool configure() = 0;
    virtual bool start_measurements() = 0;
    virtual void read_raw(struct raw_value *rv) = 0;
    virtual uint32_t get_dev_id() = 0;
};

>>>>>>> 86b3312... AP_Compass: AK8963: factor out common code of read_raw()
class AP_Compass_AK8963 : public AP_Compass_Backend
{
public:
    AP_Compass_AK8963(Compass &compass);

    static AP_Compass_Backend *detect(Compass &compass);

    bool        init(void);
    void        read(void);
    void        accumulate(void);

private:
    typedef enum
    {
        STATE_UNKNOWN,
        STATE_CONVERSION,
        STATE_SAMPLE,
        STATE_ERROR
    } state_t;

    bool _read_raw();

    bool _reset();
    bool _setup_mode();
    bool _check_id();
    bool _calibrate();

    void _update();
    bool _start_conversion();
    bool _collect_samples();
    void _dump_registers();

    bool _configure_mpu9250();
    void _bus_read(uint8_t address, uint8_t *value, uint32_t count);
    void _bus_write(uint8_t address, const uint8_t *value,  uint32_t count);

    void _bus_write(uint8_t address, const uint8_t value) {
        _bus_write(address, &value, 1);
    }

    void _register_read(uint8_t address, uint8_t *value, uint8_t count);

    uint8_t _register_read(uint8_t address) {
        uint8_t reg;
        _register_read(address, &reg, 1);
        return reg;
    }

    void _register_write(uint8_t address, uint8_t value);

    bool _sem_take_nonblocking();
    bool _sem_take_blocking();
    bool _sem_give();

    state_t             _state;

    float               _magnetometer_ASA[3] {0, 0, 0};
    uint8_t             _compass_instance;

    float               _mag_x_accum;
    float               _mag_y_accum;
    float               _mag_z_accum;
    uint32_t            _accum_count;

    bool                _initialized;
    uint32_t            _last_update_timestamp;
    uint32_t            _last_accum_time;

    AP_HAL::Semaphore   *_spi_sem;
    AP_HAL::SPIDeviceDriver   *_spi;
};

<<<<<<< HEAD
=======
class AP_AK8963_SerialBus_MPU9250: public AP_AK8963_SerialBus
{
public:
    AP_AK8963_SerialBus_MPU9250();
    void register_read(uint8_t address, uint8_t *value, uint8_t count);
    void register_write(uint8_t address, uint8_t value);
    AP_HAL::Semaphore* get_semaphore();
    bool configure();
    bool start_measurements();
    void read_raw(struct raw_value *rv);
    uint32_t get_dev_id();
private:
    void _read(uint8_t address, uint8_t *value, uint32_t count);
    void _write(uint8_t address, const uint8_t *value,  uint32_t count);
    void _write(uint8_t address, const uint8_t value) {
        _write(address, &value, 1);
    }
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
};

class AP_AK8963_SerialBus_I2C: public AP_AK8963_SerialBus
{
public:
    AP_AK8963_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr);
    void register_read(uint8_t address, uint8_t *value, uint8_t count);
    void register_write(uint8_t address, uint8_t value);
    AP_HAL::Semaphore* get_semaphore();
    bool configure(){ return true; }
    bool start_measurements() { return true; }
    void read_raw(struct raw_value *rv);
    uint32_t get_dev_id();
private:
    void _read(uint8_t address, uint8_t *value, uint32_t count);
    void _write(uint8_t address, const uint8_t *value,  uint32_t count);
    void _write(uint8_t address, const uint8_t value) {
        _write(address, &value, 1);
    }
    AP_HAL::I2CDriver *_i2c;
    AP_HAL::Semaphore *_i2c_sem;
    uint8_t _addr;
};
>>>>>>> 86b3312... AP_Compass: AK8963: factor out common code of read_raw()
#endif
