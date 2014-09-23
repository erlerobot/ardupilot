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

#include "AP_Compass_TEST.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////////////////////////

AP_Compass_TEST::AP_Compass_TEST(AP_Compass &_compass, AP_Compass::Compass_State &_state):
    AP_Compass_Backend(_compass, _state)
{
    state.product_id = AP_COMPASS_TYPE_UNKNOWN; //We will use the unknown type for the testing class
    _num_instances = 0;
    hal.console->println("TEST-DRIVER");
}



bool AP_Compass_TEST::init(void)
{
	
    _num_instances = 1;
   compass.set_field(Vector3f(1, 1, 1));
        // get device id

        _count = 0;
        _sum.zero();
        state._healthy = false;

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count == 0) {
        hal.console->printf("Failed initial compass accumulate\n");        
    }
    return true;
}

bool AP_Compass_TEST::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
        state._healthy = true;

        _count =1; 

        _sum /= _count;
        _sum *= 1000;

         _sum += state._offset.get();

        // apply motor compensation
        if (state._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && state._thr_or_curr != 0.0f) {
            state._motor_offset = state._motor_compensation.get() * state._thr_or_curr;
            _sum += state._motor_offset;
        } else {
            state._motor_offset.zero();
        }
    
        state._field = _sum;
    
        _sum.zero();
        _count = 0;

    state.last_update = _last_timestamp;
    
    return true;
}

void AP_Compass_TEST::accumulate(void)
{
            Vector3f f = compass.get_field();
            f.x++;
            f.y++;
            f.z++;

            compass.set_field(f);            
}

uint8_t AP_Compass_TEST::get_primary(void) const
{
   
    return 0;
}

