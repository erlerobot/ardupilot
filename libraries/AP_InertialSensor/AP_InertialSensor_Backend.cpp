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

#include <AP_InertialSensor.h>
#include <stdio.h>
extern const AP_HAL::HAL& hal;

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &_imu):
    //port(_port),
    imu(_imu)
{
}

void
AP_InertialSensor_Backend::init( AP_InertialSensor::Start_style style,
                         AP_InertialSensor::Sample_rate sample_rate)
{
    imu._product_id = _init_sensor(sample_rate);

    // check scaling
    for (uint8_t i=0; i<imu.get_accel_count(); i++) {
        if (imu._accel_scale[i].get().is_zero()) {
            imu._accel_scale[i].set(Vector3f(1,1,1));
        }
    }

    if (AP_InertialSensor::WARM_START != style) {
        // do cold-start calibration for gyro only
        init_gyro();
    }
}

/*
void
AP_InertialSensor_Backend::init_gyro()
{
    _init_gyro();

    // save calibration
    _save_parameters();
}*/

void
AP_InertialSensor_Backend::init_gyro()
{
    uint8_t num_gyros = min(imu.get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average[INS_MAX_INSTANCES], best_avg[INS_MAX_INSTANCES];
    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];

    // cold start
    hal.console->print_P(PSTR("Init Gyro"));

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        imu._gyro_offset[k] = Vector3f(0,0,0);
        best_diff[k] = 0;
        last_average[k].zero();
        converged[k] = false;
    }

    for(int8_t c = 0; c < 5; c++) {
        hal.scheduler->delay(5);
        _update();
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;

    // we try to get a good calibration estimate for up to 10 seconds
    // if the gyros are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 20 && num_converged < num_gyros; j++) {
        Vector3f gyro_sum[INS_MAX_INSTANCES], gyro_avg[INS_MAX_INSTANCES], gyro_diff[INS_MAX_INSTANCES];
        float diff_norm[INS_MAX_INSTANCES];
        uint8_t i;

        memset(diff_norm, 0, sizeof(diff_norm));

        hal.console->print_P(PSTR("*"));

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_sum[k].zero();
        }
        for (i=0; i<50; i++) {
            _update();
            for (uint8_t k=0; k<num_gyros; k++) {
                gyro_sum[k] += imu.get_gyro(k);
            }
            hal.scheduler->delay(5);
        }
        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_avg[k] = gyro_sum[k] / i;
            gyro_diff[k] = last_average[k] - gyro_avg[k];
            diff_norm[k] = gyro_diff[k].length();
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            if (converged[k]) continue;
            if (j == 0) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = gyro_avg[k];
            } else if (gyro_diff[k].length() < ToRad(0.1f)) {
                // we want the average to be within 0.1 bit, which is 0.04 degrees/s
                last_average[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                imu._gyro_offset[k] = last_average[k];            
                converged[k] = true;
                num_converged++;
            } else if (diff_norm[k] < best_diff[k]) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
            }
            last_average[k] = gyro_avg[k];
        }
    }

    // stop flashing leds
    AP_Notify::flags.initialising = false;

    if (num_converged == num_gyros) {
        // all OK
        return;
    }

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    hal.console->println();
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
            hal.console->printf_P(PSTR("gyro[%u] did not converge: diff=%f dps\n"), 
                                  (unsigned)k, ToDeg(best_diff[k]));
            imu._gyro_offset[k] = best_avg[k];
        }
    }
     
    imu._save_parameters();
}

/*
void
AP_InertialSensor_Backend::init_accel()
{
    _init_accel();

    // save calibration
    _save_parameters();
}*/

void
AP_InertialSensor_Backend::init_accel()
{
    uint8_t num_accels = min(imu.get_accel_count(), INS_MAX_INSTANCES);
    uint8_t flashcount = 0;
    Vector3f prev[INS_MAX_INSTANCES];
    Vector3f accel_offset[INS_MAX_INSTANCES];
    float total_change[INS_MAX_INSTANCES];
    float max_offset[INS_MAX_INSTANCES];

    memset(max_offset, 0, sizeof(max_offset));
    memset(total_change, 0, sizeof(total_change));

    // cold start
    hal.scheduler->delay(100);

    hal.console->print_P(PSTR("Init Accel"));

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // clear accelerometer offsets and scaling
    for (uint8_t k=0; k<num_accels; k++) {
        imu._accel_offset[k] = Vector3f(0,0,0);
        imu._accel_scale[k] = Vector3f(1,1,1);

        // initialise accel offsets to a large value the first time
        // this will force us to calibrate accels at least twice
        accel_offset[k] = Vector3f(500, 500, 500);
    }

    // loop until we calculate acceptable offsets
    while (true) {
        // get latest accelerometer values
        _update();

        for (uint8_t k=0; k<num_accels; k++) {
            // store old offsets
            prev[k] = accel_offset[k];

            // get new offsets
            accel_offset[k] = imu.get_accel(k);
        }

        // We take some readings...
        for(int8_t i = 0; i < 50; i++) {

            hal.scheduler->delay(20);
            _update();

            // low pass filter the offsets
            for (uint8_t k=0; k<num_accels; k++) {
                accel_offset[k] = accel_offset[k] * 0.9f + imu.get_accel(k) * 0.1f;
            }

            // display some output to the user
            if(flashcount >= 10) {
                hal.console->print_P(PSTR("*"));
                flashcount = 0;
            }
            flashcount++;
        }

        for (uint8_t k=0; k<num_accels; k++) {
            // null gravity from the Z accel
            accel_offset[k].z += GRAVITY_MSS;

            total_change[k] = 
                fabsf(prev[k].x - accel_offset[k].x) + 
                fabsf(prev[k].y - accel_offset[k].y) + 
                fabsf(prev[k].z - accel_offset[k].z);
            max_offset[k] = (accel_offset[k].x > accel_offset[k].y) ? accel_offset[k].x : accel_offset[k].y;
            max_offset[k] = (max_offset[k] > accel_offset[k].z) ? max_offset[k] : accel_offset[k].z;
        }

        uint8_t num_converged = 0;
        for (uint8_t k=0; k<num_accels; k++) {
            if (total_change[k] <= AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE && 
                max_offset[k] <= AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET) {
                num_converged++;
            }
        }

        if (num_converged == num_accels) break;

        hal.scheduler->delay(500);
    }

    // set the global accel offsets
    for (uint8_t k=0; k<num_accels; k++) {
        imu._accel_offset[k] = accel_offset[k];
    }

    imu._save_parameters();
    // stop flashing the leds
    AP_Notify::flags.initialising = false;

    hal.console->print_P(PSTR(" "));

}
