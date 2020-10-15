cmake_minimum_required(VERSION 2.8.3)
project(robomaid_2dnav)


find_package(catkin REQUIRED COMPONENTS
  roscpp
rosbag
  move_base_msgs
  actionlib
  tf
 costmap_2d
carrot_planner
     
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sendgoal src/sendgoal.cpp)
target_link_libraries(sendgoal ${catkin_LIBRARIES})

install(TARGETS sendgoal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



foreach(dir launch maps)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
