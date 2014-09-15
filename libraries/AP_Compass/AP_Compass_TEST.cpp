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

AP_Compass_TEST::AP_Compass_TEST(AP_Compass &_compass):
    AP_Compass_Backend(_compass)
{
    compass.product_id = AP_COMPASS_TYPE_UNKNOWN; //We will use the unknown type for the testing class
    _num_instances = 0;
    hal.console->println("TEST-DRIVER");
}



bool AP_Compass_TEST::init(void)
{
	
    _num_instances = 1;
   compass.set_field(Vector3f(1, 1, 1));
    for (uint8_t i=0; i<_num_instances; i++) {
        // get device id
        


        _count[0] = 0;
        _sum[i].zero();
        compass._healthy[i] = false;
    }

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count[0] == 0) {
        hal.console->printf("Failed initial compass accumulate\n");        
    }
    return true;
}

bool AP_Compass_TEST::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    accumulate();

    // consider the compass healthy if we got a reading in the last 0.2s
    for (uint8_t i=0; i<_num_instances; i++) {
        compass._healthy[i] = true;
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        // avoid division by zero if we haven't received any mag reports
        if (_count[i] == 0) continue;

        _sum[i] /= _count[i];
        _sum[i] *= 1000;

         _sum[i] += compass._offset[i].get();

        // apply motor compensation
        if (compass._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && compass._thr_or_curr != 0.0f) {
            compass._motor_offset[i] = compass._motor_compensation[i].get() * compass._thr_or_curr;
            _sum[i] += compass._motor_offset[i];
        } else {
            compass._motor_offset[i].zero();
        }
    
        compass._field[i] = _sum[i];
    
        _sum[i].zero();
        _count[i] = 0;
    }

    compass.last_update = _last_timestamp[get_primary()];
    
    return true;
}

void AP_Compass_TEST::accumulate(void)
{
   
    for (uint8_t i=0; i<_num_instances; i++) {
        
            Vector3f f = compass.get_field(i);
            f.x++;
            f.y++;
            f.z++;

            compass.set_field(f);            
    }
}

uint8_t AP_Compass_TEST::get_primary(void) const
{
   
    return 0;
}

