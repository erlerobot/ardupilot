#include "ros/ros.h"
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apm_rc");

  std::string topic_name = std::string("apm_rc");
  //std::cout << topic_name << std::endl;

  ros::NodeHandle n;
  //ros::Publisher baro_pub = n.advertise<apm_baro::apm_baro>(topic_name, 1000);
  //apm_baro::apm_baro msg;

  ros::Rate loop_rate(10);
  while (ros::ok()){
   
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}