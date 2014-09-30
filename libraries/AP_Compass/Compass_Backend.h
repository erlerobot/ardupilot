// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  Compass driver backend class
 */
#ifndef __AP_COMPASS_BACKEND_H__
#define __AP_COMPASS_BACKEND_H__

#include <AP_Compass.h>


class AP_Compass_Backend
{
public:   
    AP_Compass_Backend(AP_Compass &_compass, AP_Compass::Compass_State &_state);
    
    // we declare a virtual destructor so that Compass drivers can
    // override with a custom destructor if need be.
    virtual ~AP_Compass_Backend(void) {}

    virtual bool init() = 0;
    
    virtual bool read() = 0;

    virtual void accumulate(void) = 0;

    void learn_offsets(void);

protected:
    AP_Compass                          &compass;                        ///< access to frontend (for parameters)
    AP_Compass::Compass_State           &state;                          ///< public state for this instance
    // common utility functions
   // int32_t swap_int32(int32_t v) const;
    //int16_t swap_int16(int16_t v) const;


};
#endif //__AP_COMPASS_BACKEND_H__
