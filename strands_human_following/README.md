Package Description
=====================
This package contains simple human following

The main function of this package is making the robot follow humans using people tracker.

The State Machine inside the package contains:

  Start stage - Wandering stage - Following stage - Local-searching stage
  
Getting Start
=========================
Before you can launch this package, you need:
    
    (on bob)
    roscore
    bring_up
    
    (on bobl)
    openni_wrapper
    scitos_ptu ptu_action_server
    people_track

To start this human_following module, simply launch:
    
    roslaunch strands_human_following simple_follow.launch [wandering_mode, config_file]
    rosrun strands_human_following simple_follow_client.py
    
There are two parameters that can be changed:
* wandering_mode [wait, normal] with wait as default
* config_file where the configuration of this module is stored. It is stored in /strands_hri/strands_human_following/conf/default.yaml by default

Configurations
==========================
To change the settings of this package, find:
    
    /strands_hri/strands_human_following/conf/default.yaml
    
Parameters you can change are listed below:

    wander_area            : (polygon)
    follow_area            : (polygon)
    max_t_frames(tolerence): (int)
    alpha                  : (double)
    distance(safe distance): (double)
    
    
Problems
============================

Local Searching state does not work properly.

Can not catch up with human's normal pace.

Can not follow when people walk towards or pass by the robot.
