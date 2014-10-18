cmake_minimum_required(VERSION 2.8.3)
project(strands_visualise_speech)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
    DIRECTORY action
    FILES SoundLights.action
)


generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/sound_to_light.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
