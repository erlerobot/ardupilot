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

#include <AP_Compass.h>

// don't allow any axis of the offset to go above 2000
#define COMPASS_OFS_LIMIT 2000

AP_Compass_Backend::AP_Compass_Backend(AP_Compass &_compass, AP_Compass::Compass_State &_state):
    compass(_compass),
    state(_state)
{
}

/*
 *  this offset learning algorithm is inspired by this paper from Bill Premerlani
 *
 *  http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
 *
 *  The base algorithm works well, but is quite sensitive to
 *  noise. After long discussions with Bill, the following changes were
 *  made:
 *
 *   1) we keep a history buffer that effectively divides the mag
 *      vectors into a set of N streams. The algorithm is run on the
 *      streams separately
 *
 *   2) within each stream we only calculate a change when the mag
 *      vector has changed by a significant amount.
 *
 *  This gives us the property that we learn quickly if there is no
 *  noise, but still learn correctly (and slowly) in the face of lots of
 *  noise.
 */
void
AP_Compass_Backend::learn_offsets(void)
{
    if (state._learn == 0) {
        // auto-calibration is disabled
        return;
    }

    // this gain is set so we converge on the offsets in about 5
    // minutes with a 10Hz compass
    const float gain = 0.01;
    const float max_change = 10.0;
    const float min_diff = 50.0;

    if (!state._null_init_done) {
        // first time through
        state._null_init_done = true;
        //for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
            const Vector3f &ofs = state._offset.get();
            for (uint8_t i=0; i< state._mag_history_size; i++) {
                // fill the history buffer with the current mag vector,
                // with the offset removed
                state._mag_history[i] = Vector3i((state._field.x+0.5f) - ofs.x, (state._field.y+0.5f) - ofs.y, (state._field.z+0.5f) - ofs.z);
            }
            state._mag_history_index = 0;
        //}
        return;
    }

    //for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        const Vector3f &ofs = state._offset.get();
        Vector3f b1, diff;
        float length;

        if (ofs.is_nan()) {
            // offsets are bad possibly due to a past bug - zero them
            state._offset.set(Vector3f());
        }

        // get a past element
        b1 = Vector3f(state._mag_history[state._mag_history_index].x,
                      state._mag_history[state._mag_history_index].y,
                      state._mag_history[state._mag_history_index].z);

        // the history buffer doesn't have the offsets
        b1 += ofs;

        // get the current vector
        const Vector3f &b2 = state._field;

        // calculate the delta for this sample
        diff = b2 - b1;
        length = diff.length();
        if (length < min_diff) {
            // the mag vector hasn't changed enough - we don't get
            // enough information from this vector to use it.
            // Note that we don't put the current vector into the mag
            // history here. We want to wait for a larger rotation to
            // build up before calculating an offset change, as accuracy
            // of the offset change is highly dependent on the size of the
            // rotation.
            state._mag_history_index = (state._mag_history_index + 1) % state._mag_history_size;
            //continue;
            return;
         }

        // put the vector in the history
        state._mag_history[state._mag_history_index] = Vector3i((state._field.x+0.5f) - ofs.x, 
                                                          (state._field.y+0.5f) - ofs.y, 
                                                          (state._field.z+0.5f) - ofs.z);
        state._mag_history_index = (state._mag_history_index + 1) % state._mag_history_size;

        // equation 6 of Bills paper
        diff = diff * (gain * (b2.length() - b1.length()) / length);

        // limit the change from any one reading. This is to prevent
        // single crazy readings from throwing off the offsets for a long
        // time
        length = diff.length();
        if (length > max_change) {
            diff *= max_change / length;
        }

        Vector3f new_offsets = state._offset.get() - diff;

        if (new_offsets.is_nan()) {
            // don't apply bad offsets
           // continue;
             return;
        }

        // constrain offsets
        new_offsets.x = constrain_float(new_offsets.x, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
        new_offsets.y = constrain_float(new_offsets.y, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
        new_offsets.z = constrain_float(new_offsets.z, -COMPASS_OFS_LIMIT, COMPASS_OFS_LIMIT);
            
        // set the new offsets
        state._offset.set(new_offsets);
    //}
}

