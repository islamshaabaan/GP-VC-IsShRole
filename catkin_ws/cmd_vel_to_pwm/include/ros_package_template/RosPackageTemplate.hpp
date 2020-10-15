#pragma once

#include "ros_package_template/Algorithm.hpp"
#include <JetsonGPIO.h>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

namespace cmd_vel_to_pwm {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class cmdVelToPwm
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
	cmdVelToPwm(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~cmdVelToPwm();

 private:


  void topicCallback(const geometry_msgs::Twist& message);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;


  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  Algorithm algorithm_;
};

} /* namespace */
