cmake_minimum_required(VERSION 2.8.3)
project(strands_gazing)

find_package(catkin REQUIRED COMPONENTS 
    actionlib 
    actionlib_msgs 
    geometry_msgs 
    message_generation
    roscpp 
    sensor_msgs 
    tf 
    std_msgs 
)
find_package(Boost REQUIRED COMPONENTS thread)

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
    DIRECTORY
        action
	FILES 
	    GazeAtPose.action
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    actionlib 
    actionlib_msgs 
    geometry_msgs 
    roscpp 
    sensor_msgs 
    std_msgs 
    tf 
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(gaze_at_pose src/gaze_at_pose/gaze_at_pose.cpp)

add_dependencies(gaze_at_pose ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(gaze_at_pose
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS gaze_at_pose
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
