#include "ros_package_template/Algorithm.hpp"

namespace cmd_vel_to_pwm {

Algorithm::Algorithm()
    : average_(0.0),
      nMeasurements_(0)
{
}

Algorithm::~Algorithm()
{
}

void Algorithm::addData(const geometry_msgs::Twist data , float beta )
{
	static double velLeft =0 , velRight=0 ;
	static double newVelLeft;
	static double newVelRight;
	static double oldVelLeft=0;
	static double oldVelRight=0;
	
 	double alpha = 0;
	newVelLeft  = (data.linear.x - (data.angular.z*(0.1725 + alpha) ) ) ;
	newVelRight = (data.linear.x + (data.angular.z*(0.1725+ alpha ) ) );  
	

	
	if  (  abs(data.linear.x) <0.09 && data.angular.z !=0 ) {
	newVelLeft = (newVelLeft>0)?0.7*0.1725:(newVelLeft<0)?-0.7*0.1725:0 ;
	newVelRight = (newVelRight>0)?0.7*0.1725:(newVelRight<0)?-0.7*0.1725:0 ;

	}
else if ( data.angular.z ==0 ){

}
else{
newVelLeft = beta  * oldVelLeft + (1-beta) * newVelLeft ;
newVelRight= beta * oldVelRight + (1-beta) * newVelRight ;

}

	
	
	oldVelLeft= newVelLeft;
	oldVelRight= newVelRight;

        std::string cmd = "echo Got linear vel: "+std::to_string (data.linear.x)+" and angular: "+std::to_string (data.angular.z);
	system(cmd.c_str());
        
	GPIOToRobot(newVelLeft,newVelRight);


}


bool Algorithm::GPIOToRobot( const double velLeft , const double velRight )
{

	 GPIO::output(13, velLeft>=0?GPIO::HIGH:GPIO::LOW);
	 GPIO::output(15, velLeft<0?GPIO::HIGH:GPIO::LOW); 

	 GPIO::output(19, velRight>=0?GPIO::HIGH:GPIO::LOW);
	 GPIO::output(21, velRight<0?GPIO::HIGH:GPIO::LOW); 

	

         int dutyLeft = ((abs(velLeft) / 0.5 )* 8000)+  16500 ;
	 int dutyRight = ((abs(velRight) / 0.5 )* 8000)+  16500 +650 ;
         dutyLeft = (dutyLeft>20000)?20000:dutyLeft; 
         dutyRight = (dutyRight>20000)?20000:dutyRight;

         
	 std::string cmd2 = "echo out left duty cycle: "+std::to_string (dutyLeft)+" and right: "+std::to_string (dutyRight);
	 system(cmd2.c_str());

	 std::string strDutyRight = std::to_string (dutyRight);
	 
	

	 std::string strDutyLeft = std::to_string (dutyLeft);
	


	 std::string cmd ="cd /sys/devices/7000a000.pwm/pwm/pwmchip0/ && sudo echo "+  std::to_string (dutyRight)+" > pwm2/duty_cycle ;sudo echo "+std::to_string (dutyLeft) +" > pwm0/duty_cycle" ;
	 system(cmd.c_str());





	return true;
}




} /* namespace */
