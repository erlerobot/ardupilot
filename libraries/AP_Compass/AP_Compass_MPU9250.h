/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_MPU9250_H
#define AP_Compass_MPU9250_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"
#include "../Filter/Filter.h"
#include "../Filter/LowPassFilter2p.h"

#include <AP_Compass.h>

class AP_Compass_MPU9250 : public AP_Compass_Backend
{
private:
    //float               calibration[3];
    bool                _initialised;
    virtual bool        read_raw(void);
    uint8_t             _base_config;
    uint8_t             read_register(uint8_t reg);
    void                write_register(uint8_t address, uint8_t value);
    void                show_all_registers();
    uint32_t            _retry_time; // when unhealthy the millis() value to retry at
   
    AP_HAL::SPIDeviceDriver     *_spi;
    AP_HAL::Semaphore           *_spi_sem;

    uint8_t			    _accum_count;
    uint32_t            _last_accum_time;

    float adjust_x, adjust_y, adjust_z;    //Adjustment values

    float  mag_x, mag_y, mag_z;
    float  mag_x_accum, mag_y_accum, mag_z_accum;  //Accumulated values
/*
    // Low Pass filters for magnetometer
    LowPassFilter2p     mag_filter_x;
    LowPassFilter2p     mag_filter_y;
    LowPassFilter2p     mag_filter_z;

*/
public:
    AP_Compass_MPU9250(AP_Compass &_compass);
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

};
#endif
