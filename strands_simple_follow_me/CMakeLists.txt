cmake_minimum_required(VERSION 2.8.3)
project(strands_simple_follow_me)

find_package(catkin REQUIRED COMPONENTS 
    actionlib 
    bayes_people_tracker 
    geometry_msgs 
    move_base_msgs
    roscpp 
    tf 
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib 
    bayes_people_tracker 
    geometry_msgs 
    move_base_msgs
    roscpp 
    tf 
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_follow_me src/main.cpp)

add_dependencies(simple_follow_me 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(simple_follow_me
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS simple_follow_me
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
