cmake_minimum_required(VERSION 3.1)
project(cleanRoom)

## Find catkin macros and libraries
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs 
  rosbag
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  gridmap_2d
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    std_msgs
    visualization_msgs
    gridmap_2d
)

###########
## Build ##
###########
# c++11 support required
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(cleanRoom
  src/cleanRoom.cpp
  
)
add_dependencies(cleanRoom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cleanRoom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

#############
## Install ##
#############

# install nodes
#install(TARGETS cleanRoom
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# install roslaunch files
#install(DIRECTORY launch/
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
  #find_package(roslaunch REQUIRED)

  # test all launch files
  #roslaunch_add_file_check(launch)
#endif()

