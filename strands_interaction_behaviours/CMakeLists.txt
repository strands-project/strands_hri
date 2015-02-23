cmake_minimum_required(VERSION 2.8.3)
project(strands_interaction_behaviours)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    std_msgs
    roscpp
    rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  DIRECTORY action
  FILES
  InteractionIdle.action
  BehaviourSwitch.action
  IdleBehaviour.action
  InteractionEngaged.action
  Review.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
  actionlib_msgs 
  std_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/insert_dialogue.py
  idle/scripts/behaviour_switch/behaviour_switch.py
  idle/scripts/engaged_server/engaged_server.py
  idle/scripts/idle_behaviour/idle_behaviour.py
  idle/scripts/idle_server/idle_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
