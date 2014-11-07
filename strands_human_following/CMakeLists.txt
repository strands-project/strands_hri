cmake_minimum_required(VERSION 2.8.3)
project(strands_human_following)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  nav_goals_msgs
  rospy
  sensor_msgs
  smach
  smach_ros
  std_msgs
  tf
)

 catkin_python_setup()

 add_action_files(
   DIRECTORY action
   FILES SimpleFollow.action
 )

 generate_messages(
   DEPENDENCIES actionlib_msgs std_msgs
 )

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs nav_goals_msgs rospy sensor_msgs
  smach smach_ros std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
