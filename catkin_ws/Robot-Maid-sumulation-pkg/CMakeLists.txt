cmake_minimum_required(VERSION 2.8.3)
project(RoboMaid_sim)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    visualization_msgs
    nav_msgs
    actionlib
    move_base_msgs 
    rosbag
    std_msgs
)

catkin_package(

  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    visualization_msgs
    actionlib
    move_base_msgs 
    nav_msgs
    rosbag
    std_msgs
#  DEPENDS
#    Boost
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(watchdog src/watchdog.cpp )
add_executable(starter src/starter.cpp)
target_link_libraries(
  watchdog
  ${catkin_LIBRARIES}
)
target_link_libraries(
  starter
  ${catkin_LIBRARIES}
)
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()



