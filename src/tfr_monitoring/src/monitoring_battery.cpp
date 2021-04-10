/****************************************************************************************
 * File:    monitoring_battery.cpp
 * Node:    monitoring_battery
 *
 * Purpose: This is a subscriber that will listen to the voltage output of the
 *battery. When the battery is outputting
 *
 ****************************************************************************************/

#include "ros/ros.h"
#include <std_msgs/UInt16.h>

<<<<<<< HEAD
void batteryVoltageCallback(const std_msgs::UInt16 &batteryVoltage) {
  // Roboteq controller sends voltage * 10 back i.e. if 150 is reported than it
  // is 15 volts. What is reported is diveded by 10 to get the actual voltage.
  int batteryVolt = batteryVoltage.data / 10;
  // if battery voltage is below 37 than it needs to be charged
  if (batteryVolt < 37) {
    ROS_ERROR("BATTERY LOW! CHARGE NOW! (%d volts)\n", batteryVolt);
  }
=======


void batteryVoltageCallback(const std_msgs::UInt16& batteryVoltage) {
	//Roboteq controller sends voltage * 10 back i.e. if 150 is reported than it is 15 volts.
        int batteryVolt = batteryVoltage.data / 10;
        if (batteryVolt < 37) {
        	ROS_ERROR("BATTERY LOW! CHARGE NOW! (%d volts)\n", batteryVolt);
        }
>>>>>>> 71c30e1ea1f68124202cc84ea9c4316dfef0c354
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "monitoring_battery");
  ros::NodeHandle n;
  ros::Subscriber batteryVoltageSubscriber =
      n.subscribe("/device8/get_qry_volts/v_bat", 5, batteryVoltageCallback);
  ros::spin();
  return 1;
}
