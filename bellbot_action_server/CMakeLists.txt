cmake_minimum_required(VERSION 2.8.3)
project(bellbot_action_server)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  rospy
  smach 
  smach_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   BellbotState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  NewTarget.srv
#  Feedback.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  bellbot.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bellbot_action_server
#  CATKIN_DEPENDS rospy smach smach_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
#  launch/bellbot_action_server.launch
  scripts/bellbot_server.py
  #src/bellbot_state_machine.py
  #src/agent.py
  #scripts/bellbot_test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
