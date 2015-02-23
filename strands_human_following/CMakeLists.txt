cmake_minimum_required(VERSION 2.8.3)
project(strands_human_following)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bayes_people_tracker
  geometry_msgs
  nav_goals_generator
  rospy
  scitos_ptu
  sensor_msgs
  smach
  smach_ros
  std_msgs
  strands_navigation_msgs
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
  CATKIN_DEPENDS 
    actionlib 
    actionlib_msgs 
    bayes_people_tracker 
    geometry_msgs
    nav_goals_generator
    rospy 
    scitos_ptu 
    sensor_msgs
    smach 
    smach_ros 
    std_msgs
    strands_navigation_msgs
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/simple_follow_client.py
  scripts/simple_follow_server.py
  src/strands_human_following/__init__.py
  src/strands_human_following/human_follow.py
  src/strands_human_following/follow_and_move.py
  src/strands_human_following/local_search.py
  src/strands_human_following/wander_search.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
