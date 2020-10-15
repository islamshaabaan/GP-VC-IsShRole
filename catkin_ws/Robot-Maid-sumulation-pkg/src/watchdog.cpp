#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include <ros/callback_queue.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <thread>


using namespace std ;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool explroe_finished=false;

void explore_status(const std_msgs::Bool::ConstPtr& msg)
{
 ROS_INFO("explorer node has finished ");
 explroe_finished=msg->data;

}




int main(int argc, char **argv)
{
//    int empty_count = 0 ;  
   int no_trails = 1;
   string prefix("roslaunch cleanRoom clean.launch map_path:=");
   
   ros::init(argc,argv,"watchdog");
   ros::NodeHandle n;
 
   
   ros::Subscriber exp_status=n.subscribe("explore/status",100, explore_status); 
   ros::ServiceClient map_fetcher=n.serviceClient<nav_msgs::GetMap>("dynamic_map");
   MoveBaseClient go_home("move_base", true);
   nav_msgs::GetMap map_srv ;
   rosbag::Bag map_storage;
   ros::Rate loop_rate(1);
   



   while (ros::ok()&!explroe_finished)
     {

         ros::spinOnce();
         loop_rate.sleep();
     }

   ROS_INFO("explorer node has finished working,our Trip to home start now");

     move_base_msgs::MoveBaseGoal goal; 
     goal.target_pose.header.stamp = ros::Time::now();
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.pose.orientation.w = 1.0;
     ROS_INFO("Sending goal to move_base");

     while (no_trails<5)
      {
             go_home.sendGoal(goal);
             go_home.waitForResult();
        if(go_home.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
         {
            ROS_INFO("we arrived home");
            no_trails  =100;
         }
         else 
         { 
             ROS_INFO("arriving to home failed,trying again");
             no_trails ++;
         }
       }




    if(map_fetcher.call(map_srv))
     {
	     ROS_INFO("start saving map !");
             map_storage.open(argv[1],rosbag::bagmode::Write);
             map_storage.write<nav_msgs::OccupancyGrid>("map",ros::Time::now(),map_srv.response.map);	
             map_storage.close();
        ROS_INFO("start cleaning");
            if(system(NULL))
               system((prefix+argv[1]).c_str());
             

     }

   //ROS_INFO("hello there");
}
