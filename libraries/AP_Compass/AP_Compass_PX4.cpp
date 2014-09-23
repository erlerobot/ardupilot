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
 *       AP_Compass_PX4.cpp - Arduino Library for PX4 magnetometer
 *
 */


#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_Compass_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_device.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////////////////////////

AP_Compass_PX4::AP_Compass_PX4(AP_Compass &_compass, AP_Compass::Compass_State &_state):
    AP_Compass_Backend(_compass, _state)
{
    state.product_id = AP_COMPASS_TYPE_PX4;
    _num_instances = 0;
//    hal.console->println("PX4");
}



bool AP_Compass_PX4::init(void)
{
	_mag_fd = open(MAG_DEVICE_PATH, O_RDONLY);
	//_mag_fd = open(MAG_DEVICE_PATH "1", O_RDONLY);
	//_mag_fd = open(MAG_DEVICE_PATH "2", O_RDONLY);

    _num_instances = 0;
    //for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (_mag_fd >= 0) {
            _num_instances = i+1;
        }
    //}   
	if (_num_instances == 0) {
        hal.console->printf("Unable to open " MAG_DEVICE_PATH "\n");
        return false;
	}

   // for (uint8_t i=0; i<_num_instances; i++) {
        // get device id
        state._dev_id = ioctl(_mag_fd, DEVIOCGDEVICEID, 0);

        // average over up to 20 samples
        if (ioctl(_mag_fd, SENSORIOCSQUEUEDEPTH, 20) != 0) {
            hal.console->printf("Failed to setup compass queue\n");
            return false;                
        }

        // remember if the compass is external
        _is_external = (ioctl(_mag_fd, MAGIOCGEXTERNAL, 0) > 0);
        if (_is_external) {
            hal.console->printf("Using external compass[%u]\n", (unsigned)i);
        }
        _count = 0;
        _sum.zero();
        state._healthy = false;
    //}

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count == 0) {
        hal.console->printf("Failed initial compass accumulate\n");        
    }
    return true;
}

bool AP_Compass_PX4::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
   // for (uint8_t i=0; i<_num_instances; i++) {
        state._healthy = (hrt_absolute_time() - _last_timestamp < 200000);
    //}

    //for (uint8_t i=0; i<_num_instances; i++) {
        // avoid division by zero if we haven't received any mag reports
        if (_count == 0) continue;

        _sum /= _count;
        _sum *= 1000;

        // apply default board orientation for this compass type. This is
        // a noop on most boards
        _sum.rotate(MAG_BOARD_ORIENTATION);

        // override any user setting of COMPASS_EXTERNAL 
        state._external.set(_is_external);

        if (_is_external) {
            // add user selectable orientation
            _sum.rotate((enum Rotation)state._orientation.get());
        } else {
            // add in board orientation from AHRS
            _sum.rotate(state._board_orientation);
        }

        _sum += state._offset.get();

        // apply motor compensation
        if (state._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && state._thr_or_curr != 0.0f) {
            state._motor_offset = state._motor_compensation[i].get() * state._thr_or_curr;
            _sum += state._motor_offset;
        } else {
            state._motor_offset.zero();
        }
    
        state._field = _sum;
    
        _sum.zero();
        _count = 0;
    //}

    state.last_update = _last_timestamp;
    
    return state._healthy;
}

void AP_Compass_PX4::accumulate(void)
{
    struct mag_report mag_report;
    //for (uint8_t i=0; i<_num_instances; i++) {
        while (::read(_mag_fd, &mag_report, sizeof(mag_report)) == sizeof(mag_report) &&
               mag_report.timestamp != _last_timestamp) {
            _sum += Vector3f(mag_report.x, mag_report.y, mag_report.z);
            _count++;
            _last_timestamp = mag_report.timestamp;
        }
    //}
}

uint8_t AP_Compass_PX4::get_primary(void) const
{
    //if (state._primary < _num_instances && state._healthy) {
        return state._primary;
    //}
    /*for (uint8_t i=0; i<_num_instances; i++) {
        if (state._healthy[i]) return i;
    }    
    return 0;*/
}

#endif // CONFIG_HAL_BOARD
