#include "ros/ros.h"
//#include "apm_rc/apm_rc.h"
#include "../../../libraries/RC_Channel/RC_Channel.h"
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define NUM_CHANNELS 8

static RC_Channel rc_1(CH_1);
static RC_Channel rc_2(CH_2);
static RC_Channel rc_3(CH_3);
static RC_Channel rc_4(CH_4);
static RC_Channel rc_5(CH_5);
static RC_Channel rc_6(CH_6);
static RC_Channel rc_7(CH_7);
static RC_Channel rc_8(CH_8);
static RC_Channel *rc = &rc_1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apm_rc");

  std::string topic_name = std::string("apm_rc");
  //std::cout << topic_name << std::endl;

  ros::NodeHandle n;
  //ros::Publisher baro_pub = n.advertise<apm_baro::apm_baro>(topic_name, 1000);
  //apm_baro::apm_baro msg;

  ros::Rate loop_rate(1);
  while (ros::ok()){
	RC_Channel::set_pwm_all();
	
	for (int i=0; i<NUM_CHANNELS; i++) {
        	int value = rc[i].control_in;
		std::cout <<"RC "<<i<<" channel value is"<< value << std::endl;
		}
	
	ros::spinOnce();
	loop_rate.sleep();
  }  
  return 0;
}
