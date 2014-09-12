/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       AP_Compass_MPU9250.cpp - Arduino Library for the AK8963 magnetometer, integrated in MPU9250.
 *
 */


#include "AP_Compass_MPU9250.h"
#include "../AP_HAL_Linux/GPIO.h"

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;


/** Register Map (each of them has 8 bit width)    **/
//Read only
#define MPU_REG_WIA      0x00    // Device ID
#define MPU_REG_INFO     0X01    //Information 
#define MPU_REG_ST1      0x02    //Status 1 (Data status)
#define MPU_REG_HXL      0x03    //X-axis data LOW
#define MPU_REG_HXH      0x04    //X-axis data HIGH
#define MPU_REG_HYL      0x05    //Y-axis data LOW 
#define MPU_REG_HYH      0x06    //Y-axis data HIGH 
#define MPU_REG_HZL      0x07    //Z-axis data LOW 
#define MPU_REG_HZH      0x08    //Z-axis data HIGH 
#define MPU_REG_ST2      0x09    //Status 2 (Data status) 
//Read/Write
#define MPU_REG_CNTL_1    0x0A    //Control 1
#define MPU_REG_CNTL_2    0x0B    //Control 2
#define MPU_REG_ASTC      0x0C    //Self-test
#define MPU_REG_I2CDIS    0x0E    //I2C Disable
#define MPU_REG_ASAX      0x10    //X-axis sensitivity adjustment value (Fuse ROM)
#define MPU_REG_ASAY      0x11    //Y-axis sensitivity adjustment value (Fuse ROM)
#define MPU_REG_ASAZ      0x12    //Z-axis sensitivity adjustment value (Fuse ROM)

//Scale-value //XXX
#define MPU_MAG_SCALE   /*10*4219/8190*/    0.6 //microTesla/LSB (14 bits)- RegisterMap page 10
//#define MPU_MAG_SCALE   /*10*4219/32760*/    15// microTesla/LSB (16 bits)


//Operation modes
#define POWER_DOWN_MODE                    0x00
#define SINGLE_MEASUREMENT_MODE            0x01
#define CONTINUOUS_MEASUREMENT_MODE_1      0x02 //8Hz
#define CONTINUOUS_MEASUREMENT_MODE_2      0x05 //100Hz
#define EXT_TRIGGER_MEASUREMENT_MODE       0x04
#define SELF_TEST_MODE                     0x08
#define FUSE_ROM_ACCESS_MODE               0x0F


//Constructor
AP_Compass_MPU9250::AP_Compass_MPU9250(AP_Compass &_compass):
    AP_Compass_Backend(_compass)
{
    compass.product_id = AP_COMPASS_TYPE_MPU9250;
    _initialised = false ;
    hal.console->println("MPU9250");
    _accum_count=0;
}



//read an 8 bit register
uint8_t AP_Compass_MPU9250::read_register(uint8_t reg)
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);
    return rx[1];
}


// write_register - update a register value
void AP_Compass_MPU9250::write_register(uint8_t address, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = address;
    tx[1] = value;
    _spi->transaction(tx, rx, 2); 
    
}

void AP_Compass_MPU9250::show_all_registers(){

    uint8_t reg;
     
    reg = read_register(MPU_REG_WIA);
    hal.console->printf("WIA: %X   \n", reg);

    reg = read_register(MPU_REG_INFO);
    hal.console->printf("INFO: %X   \n", reg);

    reg = read_register(MPU_REG_ST1);
    hal.console->printf("ST1: %X   \n", reg);

    reg = read_register(MPU_REG_HXL);
    hal.console->printf("HXL: %X   \n", reg);

    reg = read_register(MPU_REG_HXH);
    hal.console->printf("HXH: %X   \n", reg);

    reg = read_register(MPU_REG_HYL);
    hal.console->printf("HYL: %X   \n", reg);

    reg = read_register(MPU_REG_HYH);
    hal.console->printf("HYH: %X   \n", reg);
        
    reg = read_register(MPU_REG_HZL);
    hal.console->printf("HZL: %X   \n", reg);

    reg = read_register(MPU_REG_HZH);
    hal.console->printf("HZH: %X   \n", reg);

    reg = read_register(MPU_REG_ST2);
    hal.console->printf("ST2: %X   \n", reg);

    reg = read_register(MPU_REG_CNTL_1);
    hal.console->printf("CNTL_1: %X   \n", reg);
    
    reg = read_register(MPU_REG_CNTL_2);
    hal.console->printf("CNTL_2: %X   \n", reg);

    reg = read_register(MPU_REG_ASTC);
    hal.console->printf("ASTC: %X   \n", reg);

    reg = read_register(MPU_REG_I2CDIS);
    hal.console->printf("I2CDIS: %X   \n", reg);

    reg = read_register(MPU_REG_ASAX);
    hal.console->printf("ASAX: %X   \n", reg);
        
    reg = read_register(MPU_REG_ASAY);
    hal.console->printf("ASAY: %X   \n", reg);

    reg = read_register(MPU_REG_ASAZ);
    hal.console->printf("ASAZ: %X   \n", reg);

    hal.scheduler->delay(500);
}

/*
//TODO The data is on 2s complement, so we have to be carfeul with that

uint16_t treat2sComplemet(uint16_t u){
    if(u != 0x0000){
        return u;
    }
    
}


void convert2floats(uint16_t ux, uint16_t uy, uint16_t uz){
    
}

*/
// Read Sensor data
bool AP_Compass_MPU9250::read_raw()
{

    uint8_t xl, xh, yl, yh, zl, zh, of;
 
    do{
//TODO try reading them all at once
   
   xl =  read_register(MPU_REG_HXL);
   xh =  read_register(MPU_REG_HXH);

   yl =  read_register(MPU_REG_HYL);
   yh =  read_register(MPU_REG_HYH);

   zl =  read_register(MPU_REG_HZL);
   zh =  read_register(MPU_REG_HZH);

    mag_x = ((int16_t)xh << 8) | xl;  // Turn the MSB and LSB into a signed 16-bit value
    mag_y = ((int16_t)yh << 8) | yl;  // Data stored as little Endian
    mag_z = ((int16_t)zh << 8) | zl;

    //TODO confevert2floats
   
   
    hal.console->printf("read: %f %f  %f", mag_x, mag_y, mag_z);
    hal.console->println("");

    mag_x = 1;//mag_x * adjust_x * MPU_MAG_SCALE;
    mag_y = 1;//mag_y * adjust_y * MPU_MAG_SCALE;
    mag_z = 1;//mag_z * adjust_z * MPU_MAG_SCALE;

    //Watch for overflow
    of =  read_register(MPU_REG_ST2);
    }while( (of | 0xFF) != 0xFF);
    
        hal.console->printf("\n of: %u \n", of);
    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_MPU9250::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
    hal.console->println("ACCUM- not init");                
        return;
    }
   uint32_t tnow = hal.scheduler->micros();
   if((tnow - _last_accum_time) < 13333){
        return;
    }
   /*if (compass._healthy[0] && _accum_count != 0 && (tnow - _last_accum_time) < 13333) {//TODO check _last_accum_time
	  return;
   }*/

    if (!_spi_sem->take(100)){
    // the bus is busy - try again later
    hal.console->println("ACCUM- bus busy");                
           return;
    }
   bool result = read_raw();

   show_all_registers();
   hal.scheduler->delay(100);
    
    _spi_sem->give();

   if (result) {
    hal.console->printf("READ: %f %f  %f", mag_x, mag_y, mag_z);
    hal.console->println("");
      mag_x_accum = mag_x_accum + mag_x,
      mag_y_accum = mag_y_accum + mag_y,
      mag_z_accum = mag_z_accum + mag_z,
     hal.console->printf("accum: %f %f  %f", mag_x_accum, mag_y_accum, mag_z_accum);
    hal.console->println("");                
	  _accum_count++;
	  if (_accum_count == 14) {
	      mag_x_accum = mag_x_accum/2;
              mag_y_accum = mag_y_accum/2;
              mag_z_accum = mag_z_accum/2;

		 _accum_count = 7;
	  }
	  _last_accum_time = tnow;
   }

}


// Read Sensor data
bool AP_Compass_MPU9250::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return false;
    hal.console->println("READ- bus busy");                
    }
    if (!compass._healthy[0]) {
        if (hal.scheduler->millis() < _retry_time) {
        hal.console->println("READ- retry time");                
            return false;
        }
        /*if (!re_initialise()) {
            _retry_time = hal.scheduler->millis() + 1000;
			_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
            return false;
        }*/
    }
    accumulate();
	//if (_accum_count == 0) {
	 //  accumulate();
	   if (!compass._healthy[0]) {
		  // try again in 1 second, and set SPI clock speed slower
		  _retry_time = hal.scheduler->millis() + 1000;
		  _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
		  return false;
	   }
	//}

	compass._field[0].x = mag_x_accum * adjust_x * MPU_MAG_SCALE / _accum_count;
	compass._field[0].y = mag_y_accum * adjust_y * MPU_MAG_SCALE / _accum_count;
	compass._field[0].z = mag_z_accum * adjust_z * MPU_MAG_SCALE/ _accum_count;
	_accum_count = 0;
    /*
	mag_x_accum = 0;
	mag_y_accum = 0;
	mag_z_accum = 0;
    */

    compass.last_update = hal.scheduler->micros(); // record time of update

    // rotate to the desired orientation
    if (compass.product_id == AP_COMPASS_TYPE_MPU9250) {
        compass._field[0].rotate(ROTATION_YAW_90);
    }

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    compass._field[0].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    compass._field[0].rotate((enum Rotation)compass._orientation.get());

    if (!compass._external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
       compass. _field[0].rotate(compass._board_orientation);
    }

    compass._field[0] += compass._offset[0].get();

    // apply motor compensation
    if(compass._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && compass._thr_or_curr != 0.0f) {
        compass._motor_offset[0] = compass._motor_compensation[0].get() * compass._thr_or_curr;
        compass._field[0] += compass._motor_offset[0];
    }else{
        compass._motor_offset[0].zero();
    }

    compass._healthy[0] = true;
    
    return true;
}



// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_MPU9250::init()
{

    if (_initialised) return true;
    

    mag_x = 0;
    mag_y = 0;
    mag_z = 0;

	mag_x_accum = 0;
    mag_y_accum = 0;
    mag_z_accum = 0;

    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU9250);
    _spi_sem = _spi->get_semaphore();

    if (!_spi_sem->take(100)){
         hal.scheduler->panic(PSTR("Failed to get MPU9250 semaphore"));
    }
    
    //write_register(MPU_REG_I2CDIS, 0x1B ); //Disable I2C on the magnetometer (cannot be undone)
    //hal.scheduler->delay(10);
    
    show_all_registers();
    hal.scheduler->delay(1000);
    /********************
    Read x,y,z adjustment values
    *********************/
    //Set Power-down mode. (MODE[3:0]=“0000”)
    write_register(MPU_REG_CNTL_1, POWER_DOWN_MODE);
    hal.scheduler->delay(10);
    
    //Enter Fuse ROM (MODE[3:0]=“0000”)
    write_register(MPU_REG_CNTL_1, FUSE_ROM_ACCESS_MODE);
    hal.scheduler->delay(10);

    //Read adjustemt values //TODO read them all in once
    float asax, asay, asaz;
    asax = read_register(MPU_REG_ASAX);
    asay = read_register(MPU_REG_ASAY);
    asaz = read_register(MPU_REG_ASAZ);
    
    hal.console->println("");
    hal.console->printf("asa: %f %f  %f", asax, asay, asaz);
    hal.console->println("");

    //We should set again the magnetometer to POWER-DOWN mode, but
    //as we are doing it on the loop below, it is not necesary.

    //Calculate the adjustment values.
    //This formula appears on the AK8963 datasheet, page 32 (ASAX, ASAY, ASAZ: Sensitivity Adjustment values)
    adjust_x = (((asax -128)*0.5)/128)+1;
    adjust_y = (((asay -128)*0.5)/128)+1;
    adjust_z = (((asaz -128)*0.5)/128)+1;


     hal.console->println("");
     hal.console->printf("adj:  %f %f  %f", adjust_x, adjust_y, adjust_z);
    hal.console->println("");
    
    //(1) Set Power-down mode. (MODE[3:0]=“0000”)
    write_register(MPU_REG_CNTL_1, POWER_DOWN_MODE);
    hal.scheduler->delay(10);


    //Set measurement mode
    write_register(MPU_REG_CNTL_1, CONTINUOUS_MEASUREMENT_MODE_2);
    hal.scheduler->delay(10);
       
    _initialised = true;
    _spi_sem->give();
	// perform an initial read
	compass._healthy[0] = true;
	read();

    /********************
    ID test
    *********************//*
    uint8_t id = read_register(MPU_REG_CNTL_1);
         hal.console->println("");  
        hal.console->printf("SPI: %X   ", id);
         hal.console->println("");    
    if(id != 0x48){
        hal.console->println("Bad magnetometer ID");
        return false;
    }*/
    return _initialised;//success;    
}


