#pragma once

#include <geometry_msgs/Twist.h>
#include <string>
#include <JetsonGPIO.h>
namespace cmd_vel_to_pwm {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Add new measurement data.
   * @param data the new data.
   */
  void addData(const geometry_msgs::Twist data , float beta );//, GPIO::PWM &p1,GPIO::PWM &p2);
  bool GPIOToRobot( const double velLeft , const double velRight );//, GPIO::PWM &p1, GPIO::PWM &p2 );

  /*!
   * Get the computed average of the data.
   * @return the average of the data.
   */
  double getAverage() ;

 private:

  //! Internal variable to hold the current average.
  double average_;

  //! Number of measurements taken.
  unsigned int nMeasurements_;
};

} /* namespace */
