// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/// @file	AP_Compass.h
#ifndef __AP_COMPASS_H__
#define __AP_COMPASS_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library


// compass product id
#define AP_COMPASS_TYPE_UNKNOWN  0x00
#define AP_COMPASS_TYPE_HIL      0x01
#define AP_COMPASS_TYPE_HMC5843  0x02
#define AP_COMPASS_TYPE_HMC5883L 0x03
#define AP_COMPASS_TYPE_PX4      0x04
#define AP_COMPASS_TYPE_VRBRAIN  0x05
#define AP_COMPASS_TYPE_MPU9250  0x06

// motor compensation types (for use with motor_comp_enabled)
#define AP_COMPASS_MOT_COMP_DISABLED    0x00
#define AP_COMPASS_MOT_COMP_THROTTLE    0x01
#define AP_COMPASS_MOT_COMP_CURRENT     0x02

// setup default mag orientation for each board type
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
# define MAG_BOARD_ORIENTATION ROTATION_ROLL_180
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
# define MAG_BOARD_ORIENTATION ROTATION_NONE
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
# define MAG_BOARD_ORIENTATION ROTATION_NONE
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
# define MAG_BOARD_ORIENTATION ROTATION_NONE
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
# define MAG_BOARD_ORIENTATION ROTATION_NONE
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
# define MAG_BOARD_ORIENTATION ROTATION_NONE
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
# define MAG_BOARD_ORIENTATION ROTATION_NONE
#else
# error "You must define a default compass orientation for this board"
#endif

/**
   maximum number of compass instances available on this platform. If more
   than 1 then redundent sensors may be available
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define COMPASS_MAX_INSTANCES 3
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#define COMPASS_MAX_INSTANCES 2
#else
#define COMPASS_MAX_INSTANCES 1
#endif

// default compass device ids for each board type to most common set-up to reduce eeprom usage
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
# define COMPASS_EXPECTED_DEV_ID  73225 // external hmc5883
# define COMPASS_EXPECTED_DEV_ID2 -1    // internal ldm303d
# define COMPASS_EXPECTED_DEV_ID3 0
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
# define COMPASS_EXPECTED_DEV_ID  0
# define COMPASS_EXPECTED_DEV_ID2 0
# define COMPASS_EXPECTED_DEV_ID3 0
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
# define COMPASS_EXPECTED_DEV_ID  0
# define COMPASS_EXPECTED_DEV_ID2 0
# define COMPASS_EXPECTED_DEV_ID3 0
#else
# define COMPASS_EXPECTED_DEV_ID  0
# define COMPASS_EXPECTED_DEV_ID2 0
# define COMPASS_EXPECTED_DEV_ID3 0
#endif

class AP_Compass_Backend;

class AP_Compass
{
public:
    /// Constructor
    ///
    AP_Compass() {
    	AP_Param::setup_object_defaults(this, var_info);
    }

    struct Compass_State{
        uint8_t instance;                             //instance number
        int16_t product_id;                         /// product id
        uint32_t last_update;                       ///< micros() time of last update

        // settable parameters
        AP_Int8 _learn;                             ///<enable calibration learning

        bool _healthy;
        Vector3f _field;                            ///< magnetic field strength

        AP_Int8 _orientation;
        AP_Vector3f _offset;
        AP_Float _declination;
        AP_Int8 _use_for_yaw;                       ///<enable use for yaw calculation
        AP_Int8 _auto_declination;                  ///<enable automatic declination code
        AP_Int8 _external;                          ///<compass is external
    #if COMPASS_MAX_INSTANCES > 1
        AP_Int8 _primary;                           ///primary instance
        AP_Int32 _dev_id;                           // device id detected at init.  saved to eeprom when offsets are saved allowing ram & eeprom values to be compared as consistency check
    #endif

        bool _null_init_done;                           ///< first-time-around flag used by offset nulling

        ///< used by offset correction
        static const uint8_t _mag_history_size = 20;
        uint8_t _mag_history_index;
        Vector3i _mag_history[_mag_history_size];

        // motor compensation
        AP_Int8     _motor_comp_type;               // 0 = disabled, 1 = enabled for throttle, 2 = enabled for current
        AP_Vector3f _motor_compensation;            // factors multiplied by throttle and added to compass outputs
        Vector3f    _motor_offset; // latest compensation added to compass
        float       _thr_or_curr;                   // throttle expressed as a percentage from 0 ~ 1.0 or current expressed in amps

        // board orientation from AHRS
        enum Rotation _board_orientation;
    };



    /// Initialize the compass device.
    ///
    /// @returns    True if the compass was initialized OK, false if it was not
    ///             found or is not functioning.
    ///
    virtual bool init();

    /// Read the compass and update the mag_ variables.
    ///
    virtual bool read(void);

    

    /// use spare CPU cycles to accumulate values from the compass if
    /// possible
    virtual void accumulate(void);

    /// Calculate the tilt-compensated heading_ variables.
    ///
    /// @param dcm_matrix			The current orientation rotation matrix
    ///
    /// @returns heading in radians
    ///
    float calculate_heading(const Matrix3f &dcm_matrix) const;

    /// Sets and saves the compass offset x/y/z values.
    ///
    /// @param  i                   compass instance
    /// @param  offsets             Offsets to the raw mag_ values.
    ///
    void set_and_save_offsets(uint8_t i, const Vector3f &offsets);

    /// Saves the current offset x/y/z values for one or all compasses
    ///
    /// @param  i                   compass instance
    ///
    /// This should be invoked periodically to save the offset values maintained by
    /// ::learn_offsets.
    ///
    void save_offsets(uint8_t i);
    void save_offsets(void);

    // return the number of compass instances
    virtual uint8_t get_count(void) const { return 1; }

    /// Return the current field as a Vector3f
    const Vector3f get_field(uint8_t i) const;
    const Vector3f get_field(void) const { return get_field(get_primary()); }

    /// Return the health of a compass
    bool healthy(uint8_t i) const;
    bool healthy(void) const { return healthy(get_primary()); }

    /// set the current field as a Vector3f
    void set_field(const Vector3f &field);

    /// Returns the current offset values
    ///
    /// @returns                    The current compass offsets.
    ///
    const Vector3f get_offsets(uint8_t i) const;
    const Vector3f get_offsets(void) const { return get_offsets(get_primary()); }

    /// Sets the initial location used to get declination
    ///
    /// @param  latitude             GPS Latitude.
    /// @param  longitude            GPS Longitude.
    ///
    void set_initial_location(int32_t latitude, int32_t longitude);

    /// Program new offset values.
    ///
    /// @param  i                   compass instance
    /// @param  x                   Offset to the raw mag_x value.
    /// @param  y                   Offset to the raw mag_y value.
    /// @param  z                   Offset to the raw mag_z value.
    ///
    void set_and_save_offsets(uint8_t i, int x, int y, int z) {
        set_and_save_offsets(i, Vector3f(x, y, z));
    }

    // learn offsets accessor
    bool learn_offsets_enabled() const;

    /// Perform automatic offset updates
    ///
    void learn_offsets(void);

    /// return true if the compass should be used for yaw calculations
    bool use_for_yaw(void) const;

    /// Sets the local magnetic field declination.
    ///
    /// @param  radians             Local field declination.
    /// @param save_to_eeprom       true to save to eeprom (false saves only to memory)
    ///
    void set_declination(float radians, bool save_to_eeprom = true);
    float get_declination() const;

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation);

    /// Set the motor compensation type
    ///
    /// @param  comp_type           0 = disabled, 1 = enabled use throttle, 2 = enabled use current
    ///
    void motor_compensation_type(const uint8_t comp_type);

    /// get the motor compensation value.
    uint8_t motor_compensation_type() const;

    /// Set the motor compensation factor x/y/z values.
    ///
    /// @param  i                   instance of compass
    /// @param  offsets             Offsets multiplied by the throttle value and added to the raw mag_ values.
    ///
    void set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor);

    /// get motor compensation factors as a vector
    const Vector3f get_motor_compensation(uint8_t i) const;
    const Vector3f get_motor_compensation(void) const { return get_motor_compensation(get_primary()); }

    /// Saves the current motor compensation x/y/z values.
    ///
    /// This should be invoked periodically to save the offset values calculated by the motor compensation auto learning
    ///
    void save_motor_compensation();

    /// Returns the current motor compensation offset values
    ///
    /// @returns                    The current compass offsets.
    ///
    const Vector3f get_motor_offsets(uint8_t i) const;
    const Vector3f get_motor_offsets(void) const { return get_motor_offsets(get_primary()); }

    /// Set the throttle as a percentage from 0.0 to 1.0
    /// @param thr_pct              throttle expressed as a percentage from 0 to 1.0
    void set_throttle(float thr_pct);

    /// Set the current used by system in amps
    /// @param amps                 current flowing to the motors expressed in amps
    void set_current(float amps);

    bool _learn_load_all(void);

    void _learn_set_and_save_all(uint8_t val);

    const uint32_t get_last_update() const;

    /// Returns True if the compasses have been configured (i.e. offsets saved)
    ///
    /// @returns                    True if compass has been configured
    ///
    bool configured(uint8_t i);
    bool configured(void);

    /// Returns the instance of the primary compass
    ///
    /// @returns                    the instance number of the primary compass
    ///
    virtual uint8_t get_primary(void) const { return 0; }

    static const struct AP_Param::GroupInfo var_info[];


private:
    Compass_State            state[COMPASS_MAX_INSTANCES];
    AP_Compass_Backend      *drivers[COMPASS_MAX_INSTANCES];

    void detect_instance(uint8_t instance);
};

#include <Compass_Backend.h>
#include <AP_Compass_HIL.h>
#include <AP_Compass_HMC5843.h>
#include <AP_Compass_PX4.h>
#include <AP_Compass_VRBRAIN.h>

//#include <AP_Compass_TEST.h>
#include <AP_Compass_MPU9250.h>

#endif

