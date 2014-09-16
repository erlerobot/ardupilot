/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f

/**
   maximum number of INS instances available on this platform. If more
   than 1 then redundent sensors may be available
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define INS_MAX_INSTANCES 3
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#define INS_MAX_INSTANCES 3
#else
#define INS_MAX_INSTANCES 1
#endif

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_InertialSensor_UserInteract.h>
/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 *
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */

class AP_InertialSensor_Backend;

class AP_InertialSensor
{
public:
    AP_InertialSensor():
    _accel(),
    _gyro(),
    _board_orientation(ROTATION_NONE)
{
    AP_Param::setup_object_defaults(this, var_info);  
    primary_instance = 0;      
}


    enum Start_style {
        COLD_START = 0,
        WARM_START
    };

    // the rate that updates will be available to the application
    enum Sample_rate {
        RATE_50HZ,
        RATE_100HZ,
        RATE_200HZ,
        RATE_400HZ
    };

#if !defined( __AVR_ATmega1280__ )
    // perform accelerometer calibration including providing user instructions
    // and feedback
    bool calibrate_accel(AP_InertialSensor_UserInteract *interact,
                                 float& trim_roll,
                                 float& trim_pitch);
    // Calibration routines borrowed from Rolfe Schmidt
    // blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
    // original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

    // _calibrate_accel - perform low level accel calibration
    virtual bool            _calibrate_accel(Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale);
    virtual void            _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
    virtual void            _calibrate_reset_matrices(float dS[6], float JS[6][6]);
    virtual void            _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
    virtual void            _calculate_trim(Vector3f accel_sample, float& trim_roll, float& trim_pitch);
#endif

    void detect_instance(uint8_t instance);

    /*Detects each instance and inits it.
     */
    bool init(Start_style style, Sample_rate sample_rate);

    /*Inits all the accelerometers
    */
    void init_accel();

    /*Inits all the gyroscopes
    */
    void init_gyro();

    /* Update the sensor data of all the instances, so that getters are nonblocking.
     * Returns a bool of whether data was updated or not.
     */
    bool update();

    /// calibrated - returns true if the accelerometers have been calibrated
    ///
    /// @note this should not be called while flying because it reads from the eeprom which can be slow
    ///
    bool calibrated();

    /// Fetch the current gyro values
    ///
    /// @returns	vector of rotational rates in radians/sec
    ///
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }
    const Vector3f     &get_gyro(void) const { return get_gyro(_get_primary_gyro()); }
    void       set_gyro(uint8_t instance, const Vector3f &gyro) {}

    // set gyro offsets in radians/sec
    const Vector3f &get_gyro_offsets(uint8_t i) const { return _gyro_offset[i]; }
    const Vector3f &get_gyro_offsets(void) const { return get_gyro_offsets(_get_primary_gyro()); }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    const Vector3f     &get_accel(void) const { return get_accel(get_primary_accel()); }
    void       set_accel(uint8_t instance, const Vector3f &accel) {}

    // multi-device interface
    bool get_gyro_health(uint8_t instance) const { return true; }
    bool get_gyro_health(void) const { return get_gyro_health(_get_primary_gyro()); }
    uint8_t get_gyro_count(void) const { return 1; };

    bool get_accel_health(uint8_t instance) const { return true; }
    bool get_accel_health(void) const { return get_accel_health(get_primary_accel()); }
    uint8_t get_accel_count(void) const { return 1; };

    // get accel offsets in m/s/s
    const Vector3f &get_accel_offsets(uint8_t i) const { return _accel_offset[i]; }
    const Vector3f &get_accel_offsets(void) const { return get_accel_offsets(get_primary_accel()); }

    // get accel scale
    const Vector3f &get_accel_scale(uint8_t i) const { return _accel_scale[i]; }
    const Vector3f &get_accel_scale(void) const { return get_accel_scale(get_primary_accel()); }

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    // override default filter frequency
    void set_default_filter(float filter_hz) {
        if (!_mpu6000_filter.load()) {
            _mpu6000_filter.set(filter_hz);
        }
    }

    // get_filter - return filter in hz
    uint8_t get_filter() const { return _mpu6000_filter.get(); }

    uint16_t error_count(void) const { return 0; }
    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    uint8_t get_primary_accel(void) const { return primary_instance; }
    uint8_t _get_primary_gyro(void) const { return primary_instance; }

    bool wait_for_sample(uint16_t timeout_ms);

    float get_delta_time() const;

    float get_gyro_drift_rate(void);

    // save parameters to eeprom
    void  _save_parameters();

    // Most recent accelerometer reading obtained by ::_update
    Vector3f _accel[INS_MAX_INSTANCES];

    // previous accelerometer reading obtained by ::_update
    Vector3f _previous_accel[INS_MAX_INSTANCES];

    // Most recent gyro reading obtained by ::_update
    Vector3f _gyro[INS_MAX_INSTANCES];

    // product id
    AP_Int16 _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f             _accel_scale[INS_MAX_INSTANCES];
    AP_Vector3f             _accel_offset[INS_MAX_INSTANCES];
    AP_Vector3f             _gyro_offset[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int8                 _mpu6000_filter;

    // board orientation from AHRS
    enum Rotation     _board_orientation;


private:
    AP_InertialSensor_Backend *drivers[INS_MAX_INSTANCES];

    /// primary IMU instance
    uint8_t primary_instance;

};

#include <AP_InertialSensor_Backend.h>
#include <AP_InertialSensor_Oilpan.h>
#include <AP_InertialSensor_MPU6000.h>
#include <AP_InertialSensor_HIL.h>
#include <AP_InertialSensor_PX4.h>
#include <AP_InertialSensor_VRBRAIN.h>
#include <AP_InertialSensor_UserInteract_Stream.h>
#include <AP_InertialSensor_UserInteract_MAVLink.h>
#include <AP_InertialSensor_Flymaple.h>
#include <AP_InertialSensor_L3G4200D.h>
#include <AP_InertialSensor_MPU9150.h>
#include <AP_InertialSensor_MPU9250.h>
#include <AP_InertialSensor_L3GD20.h>

#endif // __AP_INERTIAL_SENSOR_H__
