cmake_minimum_required(VERSION 2.8.3)
project(bellbot_scheduler)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS mongodb_store roscpp rospy std_msgs sensor_msgs message_generation)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

#add_message_files(
#    FILES
#    )

#add_service_files(
#    FILES
#    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime sensor_msgs std_msgs 
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    )



