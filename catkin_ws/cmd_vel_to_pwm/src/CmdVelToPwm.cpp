#include "ros_package_template/RosPackageTemplate.hpp" // TODO change header name


#include <string>

namespace cmd_vel_to_pwm {



cmdVelToPwm::cmdVelToPwm(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{



std::string cmd = " sudo busybox devmem 0x700031fc 32 0x45;sudo busybox devmem 0x6000d504 32 0x2 ;sudo busybox devmem 0x70003248 32 0x46;sudo busybox devmem 0x6000d100 32 0x00 ;" ; 
system(cmd.c_str());

cmd = "cd /sys/devices/7000a000.pwm/pwm/pwmchip0/ &&  sudo echo 0 > export &&  sleep 0.5 &&  sudo echo 30000 > pwm0/period  && sudo echo 0 > pwm0/duty_cycle &&  sudo echo 1 > pwm0/enable "; 
system(cmd.c_str());

cmd = "cd /sys/devices/7000a000.pwm/pwm/pwmchip0/ &&  sudo echo 2 > export &&  sleep 0.5 &&  sudo echo 30000 > pwm2/period  && sudo echo 0 > pwm2/duty_cycle &&  sudo echo 1 > pwm2/enable "; 
system(cmd.c_str());




        GPIO::setmode(GPIO::BOARD);
	GPIO::setup(15, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(13, GPIO::OUT, GPIO::HIGH);

	GPIO::setup(19, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(21, GPIO::OUT, GPIO::HIGH);

  subscriber_ = nodeHandle_.subscribe("/RoboMaid_diff_drive_controller/cmd_vel", 15,  &cmdVelToPwm::topicCallback, this);


    ROS_INFO("Successfuly launched pwm node.");
}

cmdVelToPwm::~cmdVelToPwm()
{

//disabling pwm on exit

std::string cmd ="cd /sys/devices/7000a000.pwm/pwm/pwmchip0/ && sudo echo 0 > pwm2/duty_cycle && sudo echo 0 > pwm0/duty_cycle  && sudo echo 0 > unexport ; sudo echo 2 > unexport";
system(cmd.c_str());


GPIO::cleanup();
	
}



void cmdVelToPwm::topicCallback(const geometry_msgs::Twist& message)
{
algorithm_.addData(message , 0.5);
}



} /* namespace */
