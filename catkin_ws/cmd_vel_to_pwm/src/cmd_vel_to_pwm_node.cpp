#include <ros/ros.h>
#include "ros_package_template/RosPackageTemplate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_to_pwm5");
  ros::NodeHandle nodeHandle("~");

  cmd_vel_to_pwm::cmdVelToPwm cmd(nodeHandle);

  ros::spin();
  return 0;
}
