/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_Progmem.h>
#include "AP_Compass.h"
#include <stdio.h>
const AP_Param::GroupInfo AP_Compass::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: OFS_X
    // @DisplayName: Compass offsets on the X axis
    // @Description: Offset to be added to the compass x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Y
    // @DisplayName: Compass offsets on the Y axis
    // @Description: Offset to be added to the compass y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Z
    // @DisplayName: Compass offsets on the Z axis
    // @Description: Offset to be added to the compass z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS",    1, AP_Compass, state[0]._offset, 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: Radians
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, AP_Compass, state[0]._declination, 0),

    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, AP_Compass, state[0]._learn, 1), // true if learning calibration

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    4, AP_Compass, state[0]._use_for_yaw, 1), // true if used for DCM yaw

#if !defined( __AVR_ATmega1280__ )
    // @Param: AUTODEC
    // @DisplayName: Auto Declination
    // @Description: Enable or disable the automatic calculation of the declination based on gps location
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTODEC",5, AP_Compass, state[0]._auto_declination, 1),
#endif

    // @Param: MOTCT
    // @DisplayName: Motor interference compensation type
    // @Description: Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
    // @Values: 0:Disabled,1:Use Throttle,2:Use Current
    // @Increment: 1
    AP_GROUPINFO("MOTCT",    6, AP_Compass, state[0]._motor_comp_type, AP_COMPASS_MOT_COMP_DISABLED),

    // @Param: MOT_X
    // @DisplayName: Motor interference compensation for body frame X axis
    // @Description: Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Y
    // @DisplayName: Motor interference compensation for body frame Y axis
    // @Description: Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Z
    // @DisplayName: Motor interference compensation for body frame Z axis
    // @Description: Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT",    7, AP_Compass, state[0]._motor_compensation, 0),

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw136,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270
    AP_GROUPINFO("ORIENT", 8, AP_Compass, state[0]._orientation, ROTATION_NONE),

    // @Param: EXTERNAL
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk, but must be set correctly on an APM2. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option
    // @Values: 0:Internal,1:External
    // @User: Advanced
    AP_GROUPINFO("EXTERNAL", 9, AP_Compass, state[0]._external, 0),

#if COMPASS_MAX_INSTANCES > 1
    AP_GROUPINFO("OFS2",    10, AP_Compass, state[1]._offset, 0),
    AP_GROUPINFO("MOT2",    11, AP_Compass, state[1]._motor_compensation, 0),

    // @Param: PRIMARY
    // @DisplayName: Choose primary compass
    // @Description: If more than one compass is available this selects which compass is the primary. Normally 0=External, 1=Internal. If no External compass is attached this parameter is ignored
    // @Values: 0:FirstCompass,1:SecondCompass
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 12, Compass, state[1]._primary, 0),
#endif

#if COMPASS_MAX_INSTANCES > 2
    AP_GROUPINFO("OFS3",    13, Compass, state[2]._offset, 0),
    AP_GROUPINFO("MOT3",    14, Compass, state[2]._motor_compensation, 0),
#endif

#if COMPASS_MAX_INSTANCES > 1
    // @Param: DEV_ID
    // @DisplayName: Compass device id
    // @Description: Compass device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID",  15, Compass, state[0]._dev_id, COMPASS_EXPECTED_DEV_ID),

    // @Param: DEV_ID2
    // @DisplayName: Compass2 device id
    // @Description: Second compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID2", 16, Compass, state[1]._dev_id, COMPASS_EXPECTED_DEV_ID2),
#endif

#if COMPASS_MAX_INSTANCES > 2
    // @Param: DEV_ID3
    // @DisplayName: Compass3 device id
    // @Description: Third compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID3", 17, Compass, state[2]._dev_id, COMPASS_EXPECTED_DEV_ID3),
#endif

    AP_GROUPEND
};

//Method to detect new instances and create objects
void
AP_Compass::detect_instance(uint8_t instance)
{
    AP_Compass_Backend *new_compass = NULL;

    state[instance].instance = instance;
    state[instance]._board_orientation = ROTATION_NONE;

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    new_compass = new AP_Compass_VRBRAIN(*this, state[instance]);
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 && COMPASS_EXPECTED_DEV_ID ==73225 //HMC5843
    new_compass = new AP_Compass_HMC5843(*this, state[instance]);
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4 && COMPASS_EXPECTED_DEV_ID != 73225 //PX4
    new_compass = new AP_Compass_PX4(*this, state[instance]);
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    new_compass = new AP_Compass_MPU9250(*this, state[instance]);
//    new_compass = new AP_Compass_TEST(*this, state[instance]); //TEST-DRIVER, we will use it when the compass type is unknown
#else
    #error Unrecognized COMPASS
#endif
 
	if (new_compass != NULL) {
        drivers[instance] = new_compass;
        drivers[instance]->init();
	}
}


// Default init method, just returns success.
//
bool
AP_Compass::init()
{
   for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++){
        if(drivers[i] != NULL){
            drivers[i]->init();
        }
   else{
            detect_instance(i);
        }
    }
    return true;
}

bool 
AP_Compass::read(){
     for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if(drivers[i] != NULL){
            drivers[i]->read();
        }
        else{
            detect_instance(i);
        }
    }
    return true;
}

void 
AP_Compass::accumulate(){
     for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if(drivers[i] != NULL){
            drivers[i]->accumulate();
        }
        else{
            detect_instance(i);
        }
    }
}
void
AP_Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        state[i]._offset.set(offsets);
        save_offsets(i);
    }
}

void
AP_Compass::save_offsets(uint8_t i)
{
    state[i]._offset.save();  // save offsets
#if COMPASS_MAX_INSTANCES > 1
    state[i]._dev_id.save();  // save device id corresponding to these offsets
#endif
}

void
AP_Compass::save_offsets(void)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        save_offsets(i);
    }
}

void
AP_Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
    state[i]._motor_compensation.set(motor_comp_factor);
}

void
AP_Compass::save_motor_compensation()
{
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        state[k]._motor_comp_type.save();
        state[k]._motor_compensation.save();
    }
}

void
AP_Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
#if !defined( __AVR_ATmega1280__ )
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        if (state[k]._auto_declination) {
            // Set the declination based on the lat/lng from GPS
            state[k]._declination.set(radians(
                    AP_Declination::get_declination(
                        (float)latitude / 10000000,
                        (float)longitude / 10000000)));
        }
    }
#endif
}

void
AP_Compass::set_declination(float radians, bool save_to_eeprom)
{
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        if (save_to_eeprom) {
            state[k]._declination.set_and_save(radians);
        }else{
            state[k]._declination.set(radians);
        }
    }
}


float
AP_Compass::get_declination() const
{
    return state[get_primary()]._declination.get();
}



//calculate a compass heading given the attitude from DCM and the mag vector

float
AP_Compass::calculate_heading(const Matrix3f &dcm_matrix) const
{
    //TODO all sensors values instead of primarys "field"

    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);


    // Tilt compensated magnetic field Y component:
    for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++){
        Vector3f field = get_field(i);
        printf("I=%d    %f  %f  %f\n",i, field.x, field.y, field.z);
    }
    float headY = state[2]._field.y * dcm_matrix.c.z - state[2]._field.z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = state[get_primary()]._field.x * cos_pitch_sq - dcm_matrix.c.x * (state[get_primary()]._field.y * dcm_matrix.c.y + state[get_primary()]._field.z * dcm_matrix.c.z);

        printf("headY: %f   headX: %f\n",headY, headX);
    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(state[get_primary()]._declination) > 0.0f )
    {
        heading = heading + state[get_primary()]._declination;
        if (heading > PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * PI);
        else if (heading < -PI)
            heading += (2.0f * PI);
    }

    return heading;
}


/// Returns True if the compasses have been configured (i.e. offsets saved)
///
/// @returns                    True if compass has been configured
///
bool AP_Compass::configured(uint8_t i)
{
    // exit immediately if instance is beyond the number of compasses we have available
    if (i > get_count()) {
        return false;
    }

    // exit immediately if all offsets are zero
    if (get_offsets(i).length() == 0.0f) {
        return false;
    }

#if COMPASS_MAX_INSTANCES > 1
    // backup detected dev_id
    int32_t dev_id_orig = state[i]._dev_id;

    // load dev_id from eeprom
    state[i]._dev_id.load();

    // if different then the device has not been configured
    if ( state[i]._dev_id != dev_id_orig) {
        // restore device id
         state[i]._dev_id = dev_id_orig;
        // return failure
        return false;
    }
#endif

    // if we got here then it must be configured
    return true;
}

bool  AP_Compass::learn_offsets_enabled() const 
{
     bool success = true;
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        success = success && state[k]._learn;
    } 
    return success; 
}

bool AP_Compass::configured(void)
{
    bool all_configured = true;
    for(uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && configured(i);
    }
    return all_configured;
}


const Vector3f AP_Compass::get_field(uint8_t i) const
{ 
    return state[i]._field; 
}

bool AP_Compass::healthy(uint8_t i) const 
{ 
    return state[i]._healthy; 
}

void AP_Compass::set_field(const Vector3f &field) 
{ 
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        state[k]._field = field; 
    }
}

void AP_Compass::set_board_orientation(enum Rotation orientation) {
     for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        state[k]._board_orientation = orientation;
    }
}

const Vector3f AP_Compass::get_offsets(uint8_t i) const 
{ 
    return state[i]._offset; 
}

bool AP_Compass::use_for_yaw(void) const 
{
    bool success = true;
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) { 
        success = success && state[k]._healthy && state[k]._use_for_yaw;
    }           

    return success; 
}

void AP_Compass::motor_compensation_type(const uint8_t comp_type) {
    if (state[get_primary()]._motor_comp_type <= AP_COMPASS_MOT_COMP_CURRENT && state[get_primary()]._motor_comp_type != (int8_t)comp_type) {
        state[get_primary()]._motor_comp_type = (int8_t)comp_type;
        state[get_primary()]._thr_or_curr = 0;                               // set current current or throttle to zero
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            set_motor_compensation(i, Vector3f(0,0,0)); // clear out invalid compensation vectors
        }
    }
}


const Vector3f AP_Compass::get_motor_compensation(uint8_t i) const
{ 
    return state[i]._motor_compensation; 
}

const Vector3f AP_Compass::get_motor_offsets(uint8_t i) const
{ 
    return state[i]._motor_offset; 
}

void AP_Compass::set_throttle(float thr_pct) 
{
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        if(state[k]._motor_comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
            state[k]._thr_or_curr = thr_pct;
        }
   }
}

void AP_Compass::set_current(float amps) 
{
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        if(state[k]._motor_comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
            state[k]._thr_or_curr = amps;
        }
    }
}

//TODO primary's value??
uint8_t AP_Compass::motor_compensation_type() const 
{
    return state[get_primary()]._motor_comp_type;
}

void AP_Compass::learn_offsets(void)
{
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        drivers[k]->learn_offsets();
    }
}

bool AP_Compass::_learn_load_all(void)
{
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        if(!state[k]._learn.load())
            return false;
    }
    return true;
}

void AP_Compass::_learn_set_and_save_all(uint8_t val)
{
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        state[k]._learn.set_and_save(val);
    }
}

//TODO primary's value??
const uint32_t AP_Compass::get_last_update() const 
{
    return state[get_primary()].last_update;
}



