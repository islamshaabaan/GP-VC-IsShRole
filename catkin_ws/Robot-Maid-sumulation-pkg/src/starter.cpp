#include <stdlib.h>
#include "ros/ros.h"
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include "nav_msgs/OccupancyGrid.h"
#include <thread> 
using namespace std ;

int main(int argc, char **argv)
{
   ros::init(argc,argv,"starter");
   rosbag::Bag map;



try
  {
   map.open(argv[1]);
   map.close(); 
   ROS_INFO("cleaning");
   string prefix("roslaunch  robomaid_2dnav roomba_nev.launch clean_mode:=\"true\" map_path:=");
   
   system((prefix+argv[1]).c_str());
   
  }

catch(rosbag::BagIOException e)
 {

  ROS_INFO("exploring");
  string prefix("roslaunch  robomaid_2dnav roomba_nev.launch clean_mode:=\"false\" map_path:=");
  system((prefix+argv[1]).c_str());
 }
   

     

}
