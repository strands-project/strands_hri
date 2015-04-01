cmake_minimum_required(VERSION 2.8.3)
project(strands_human_aware_navigation)

find_package(catkin REQUIRED COMPONENTS
    bayes_people_tracker
    dynamic_reconfigure
    geometry_msgs
    message_filters
    roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

generate_dynamic_reconfigure_options(
  cfg/HumanAwareNavigation.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    bayes_people_tracker
    geometry_msgs
    message_filters
    roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(human_aware_cmd_vel src/main.cpp)

add_dependencies(human_aware_cmd_vel
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(human_aware_cmd_vel
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/human_aware_navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS human_aware_cmd_vel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
