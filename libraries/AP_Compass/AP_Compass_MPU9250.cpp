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
//Registers to handle the I2C slave
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_EXT_SENS_DATA_00    0x49 //First register address that stores the data retrieved by the I2C slave

//Flags to specify to the I2C slave that the operation will be a reading (write operation donot need flag)
#define READ_FLAG   0x80;

//Read only
#define MPUREG_WIA      0x00    // Device ID
#define MPUREG_INFO     0X01    //Information 
#define MPUREG_ST1      0x02    //Status 1 (Data status)
#define MPUREG_HXL      0x03    //X-axis data LOW
#define MPUREG_HXH      0x04    //X-axis data HIGH
#define MPUREG_HYL      0x05    //Y-axis data LOW 
#define MPUREG_HYH      0x06    //Y-axis data HIGH 
#define MPUREG_HZL      0x07    //Z-axis data LOW 
#define MPUREG_HZH      0x08    //Z-axis data HIGH 
#define MPUREG_ST2      0x09    //Status 2 (Data status) 
//Read/Write
#define MPUREG_CNTL_1    0x0A    //Control 1
#define MPUREG_CNTL_2    0x0B    //Control 2
#define MPUREG_ASTC      0x0C    //Self-test (we will use it as the I2C addres as well)
#define MPUREG_I2CDIS    0x0E    //I2C Disable 
#define MPUREG_ASAX      0x10    //X-axis sensitivity adjustment value (Fuse ROM)
#define MPUREG_ASAY      0x11    //Y-axis sensitivity adjustment value (Fuse ROM)
#define MPUREG_ASAZ      0x12    //Z-axis sensitivity adjustment value (Fuse ROM)

//Scale-value //XXX
#define MPU_MAG_SCALE     0.15f   // microTesla/LSB (16 bits)


//Operation modes
#define POWER_DOWN_MODE                    0x00
#define SINGLE_MEASUREMENT_MODE            0x01
#define CONTINUOUS_MEASUREMENT_MODE_1      0x02 //8Hz
#define CONTINUOUS_MEASUREMENT_MODE_2      0x05 //100Hz
#define EXT_TRIGGER_MEASUREMENT_MODE       0x04
#define SELF_TEST_MODE                     0x08
#define FUSE_ROM_ACCESS_MODE               0x0F



//Constructor
AP_Compass_MPU9250::AP_Compass_MPU9250(AP_Compass &_compass, AP_Compass::Compass_State &_state):
    AP_Compass_Backend(_compass, _state)
{
    state.product_id = AP_COMPASS_TYPE_MPU9250;
    _initialised = false ;
    hal.console->println("MPU9250");
    _accum_count=0;
}



//read an 8 bit register
uint8_t AP_Compass_MPU9250::read_register(uint8_t reg)
{
    uint8_t addr = reg | READ_FLAG; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);
    return rx[1];
}

void AP_Compass_MPU9250::read_registers( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;

    unsigned char tx[255] = {0};
	unsigned char rx[255] = {0};

	tx[0] = ReadAddr | READ_FLAG;

    _spi->transaction(tx, rx, Bytes+1);

    for(i=0; i<Bytes; i++)
    	ReadBuf[i] = rx[i + 1];

    hal.scheduler->delay(50);
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

/*
void AP_Compass_MPU9250::show_all_registers(){

    uint8_t reg;
     
    reg = read_register(MPUREG_WIA);
    hal.console->printf("WIA: %X   \n", reg);

    reg = read_register(MPUREG_INFO);
    hal.console->printf("INFO: %X   \n", reg);

    reg = read_register(MPUREG_ST1);
    hal.console->printf("ST1: %X   \n", reg);

    reg = read_register(MPUREG_HXL);
    hal.console->printf("HXL: %X   \n", reg);

    reg = read_register(MPUREG_HXH);
    hal.console->printf("HXH: %X   \n", reg);

    reg = read_register(MPUREG_HYL);
    hal.console->printf("HYL: %X   \n", reg);

    reg = read_register(MPUREG_HYH);
    hal.console->printf("HYH: %X   \n", reg);
        
    reg = read_register(MPUREG_HZL);
    hal.console->printf("HZL: %X   \n", reg);

    reg = read_register(MPUREG_HZH);
    hal.console->printf("HZH: %X   \n", reg);

    reg = read_register(MPUREG_ST2);
    hal.console->printf("ST2: %X   \n", reg);

    reg = read_register(MPUREG_CNTL_1);
    hal.console->printf("CNTL_1: %X   \n", reg);
    
    reg = read_register(MPUREG_CNTL_2);
    hal.console->printf("CNTL_2: %X   \n", reg);

    reg = read_register(MPUREG_ASTC);
    hal.console->printf("ASTC: %X   \n", reg);

    reg = read_register(MPUREG_I2CDIS);
    hal.console->printf("I2CDIS: %X   \n", reg);

    reg = read_register(MPUREG_ASAX);
    hal.console->printf("ASAX: %X   \n", reg);
        
    reg = read_register(MPUREG_ASAY);
    hal.console->printf("ASAY: %X   \n", reg);

    reg = read_register(MPUREG_ASAZ);
    hal.console->printf("ASAZ: %X   \n", reg);

    hal.scheduler->delay(500);
}*/

/*
The data is on 2s complement, so we have to be carfeul with that

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
    uint8_t buffer[7];
    uint8_t val = MPUREG_ASTC | READ_FLAG; // Set most significant bit
    
    if (!_spi_sem->take(100)){
        // the bus is busy - try again later
        hal.console->println("read_raw- bus busy");                
        return false;
    }

    write_register(MPUREG_I2C_SLV0_ADDR, val); //Set the I2C slave addres of AK8963 and set for read.

    write_register(MPUREG_I2C_SLV0_REG, MPUREG_HXL); //I2C slave 0 register address from where to begin data transfer

    write_register(MPUREG_I2C_SLV0_CTRL, 0x87); //Tell that we will be reading 6 bytes from the magnetometer

    //hal.scheduler->delay(100);
    read_registers(MPUREG_EXT_SENS_DATA_00,buffer,6);
    
    _spi_sem->give();

    mag_x = ((int16_t)buffer[1] << 8) | buffer[0];  // Turn the MSB and LSB into a signed 16-bit value
    mag_y = ((int16_t)buffer[3] << 8) | buffer[2];  // Data stored as little Endian
    mag_z = ((int16_t)buffer[5] << 8) | buffer[4];

    hal.console->printf("read: %"PRIu16" %"PRIu16"  %"PRIu16"", mag_x, mag_y, mag_z);
    hal.console->println("");

    mag_x = mag_x ;//* adjust_x * MPU_MAG_SCALE;
    mag_y = mag_y ;//* adjust_y * MPU_MAG_SCALE;
    mag_z = mag_z ;//* adjust_z * MPU_MAG_SCALE;

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

    
   bool result = read_raw();

   //show_all_registers();
  // hal.scheduler->delay(100);
    


   if (result) {
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
    if (!state._healthy) {
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
	   if (!state._healthy) {
		  // try again in 1 second, and set SPI clock speed slower
		  _retry_time = hal.scheduler->millis() + 1000;
		  _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
		  return false;
	   }
	//}

	state._field.x = mag_x_accum / _accum_count;
	state._field.y = mag_y_accum / _accum_count;
	state._field.z = mag_z_accum / _accum_count;
	_accum_count = 0;
    
	mag_x_accum = 0;
	mag_y_accum = 0;
	mag_z_accum = 0;
    

    state.last_update = hal.scheduler->micros(); // record time of update

    // rotate to the desired orientation
    if (state.product_id == AP_COMPASS_TYPE_MPU9250) {
        state._field.rotate(ROTATION_YAW_90);
    }

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    state._field.rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    state._field.rotate((enum Rotation)state._orientation.get());

    if (!state._external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
       state. _field.rotate(state._board_orientation);
    }

    state._field += state._offset.get();

    // apply motor compensation
    if(state._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && state._thr_or_curr != 0.0f) {
        state._motor_offset = state._motor_compensation.get() * state._thr_or_curr;
        state._field += state._motor_offset;
    }else{
        state._motor_offset.zero();
    }

    state._healthy = true;
    
    return true;
}



// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_MPU9250::init()
{
    uint8_t buffer[3];
    uint8_t val;

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
    
    //write_register(MPUREG_I2CDIS, 0x1B ); //Disable I2C on the magnetometer (cannot be undone)
    //hal.scheduler->delay(10);
    
    hal.scheduler->delay(1000);
    /********************
    Read x,y,z adjustment values
    ********************/

    //Set Power-down mode. (MODE[3:0]=“0000”)
    write_register(MPUREG_CNTL_1, POWER_DOWN_MODE);
    hal.scheduler->delay(10);
    
    //Enter Fuse ROM (MODE[3:0]=“0000”)
    write_register(MPUREG_CNTL_1, FUSE_ROM_ACCESS_MODE);
    hal.scheduler->delay(10);
    
    val = MPUREG_ASTC | READ_FLAG; // Set most significant bit

    write_register(MPUREG_I2C_SLV0_ADDR,val); //Set the I2C slave addres of AK8963 and set for read.
    write_register(MPUREG_I2C_SLV0_REG, MPUREG_ASAX); //I2C slave 0 register address from where to begin data transfer
    write_register(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer

    read_registers(MPUREG_EXT_SENS_DATA_00,buffer,3);

    hal.console->println("");
    hal.console->printf("asa: %f %f  %f", buffer[0], buffer[1], buffer[1]);
    hal.console->println("");

    //We should set again the magnetometer to POWER-DOWN mode, but
    //as we are doing it on the loop below, it is not necesary.

    //Calculate the adjustment values.
    //This formula appears on the AK8963 datasheet, page 32 (ASAX, ASAY, ASAZ: Sensitivity Adjustment values)
    adjust_x = ((((buffer[0] -128)*0.5)/128)+1)*MPU_MAG_SCALE;
    adjust_y = ((((buffer[1] -128)*0.5)/128)+1)*MPU_MAG_SCALE;
    adjust_z = ((((buffer[2] -128)*0.5)/128)+1)*MPU_MAG_SCALE;


     hal.console->println("");
     hal.console->printf("adj:  %f %f  %f", adjust_x, adjust_y, adjust_z);
    hal.console->println("");
    
    //(1) Set Power-down mode. (MODE[3:0]=“0000”)
    write_register(MPUREG_CNTL_1, POWER_DOWN_MODE);
    hal.scheduler->delay(10);


    //Set measurement mode
    write_register(MPUREG_CNTL_1, CONTINUOUS_MEASUREMENT_MODE_2);
    hal.scheduler->delay(10);
       

    _spi_sem->give();
	// perform an initial read

    _initialised = true;
	state._healthy = true;
	read();

    /********************
    ID test
    *********************//*
    uint8_t id = read_register(MPUREG_CNTL_1);
         hal.console->println("");  
        hal.console->printf("SPI: %X   ", id);
         hal.console->println("");    
    if(id != 0x48){
        hal.console->println("Bad magnetometer ID");
        return false;
    }*/
    return _initialised;//success;    
}


