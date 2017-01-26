#include "ros/ros.h"
#include "apm_inertial_sensor/apm_imu.h"
#include <AP_InertialSensor.h>
#include <AP_HAL.h>
#include <AP_HAL_Linux.h>
#include <iostream>
#include <cstdlib>
#include <string>

//const HAL_Linux AP_HAL_Linux;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor ins;
apm_inertial_sensor::apm_imu msg;
ros::Publisher imu_pub;

void callback(const ros::TimerEvent&){
    //ins.wait_for_sample();
    // Update IMU values
    ins.update();
    // gyro = ins.get_gyro();
    // accel = ins.get_accel();

    // Fetch the data from the imu abstraction
    msg.gyro = {ins.get_gyro().x, ins.get_gyro().y, ins.get_gyro().z};
    msg.accel = {ins.get_accel().x, ins.get_accel().y, ins.get_accel().z};

    imu_pub.publish(msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "apm_inertial_sensor");

  std::string topic_name = std::string("apm_inertial_sensor");

  ros::NodeHandle n;
  imu_pub = n.advertise<apm_inertial_sensor::apm_imu>(topic_name, 1000);

  // init APM Linux HAL
  hal.init(argc, argv);
  hal.scheduler->system_initialized();

  // init the IMU
  ins.init(AP_InertialSensor::WARM_START, AP_InertialSensor::RATE_100HZ);
  // Read sensors data 100 times in a second

  ros::Timer timer = n.createTimer(ros::Duration(0.1), callback);
  // Execute callback function every 0.1 seconds, change Duration(x) to adapt it for your purpose

  ros::spin();

  return 0;
}
