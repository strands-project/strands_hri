#! /usr/bin/env python

import roslib; roslib.load_manifest('bellbot_action_server')
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

import bellbot_action_server.msg

def bellbot_test_client(target):
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('bellbot_action_server', bellbot_action_server.msg.bellbotAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo('Waiting for server')
    client.wait_for_server()
    rospy.loginfo('Connected to server')


    # Creates a goal to send to the action server.

    goal = bellbot_action_server.msg.bellbotGoal(mode=1, starting_waypoint_name = target, preselected_goal="WayPoint2",text="STRANDS Winter School")
    #goal = bellbot_action_server.msg.bellbotGoal(mode=2, starting_waypoint_name = target, preselected_goal="WayPoint2",text="Welcome to the STRANDS Winter School. Are you ready to code?")
    #goal = bellbot_action_server.msg.bellbotGoal(mode=3, starting_waypoint_name = target, preselected_goal="WayPoint2",text="Welcome Mr. X.'")

    # Sends the goal to the action server.
    rospy.loginfo('Sending goal')
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    rospy.loginfo('Waiting for result')
    client.wait_for_result()
    rospy.loginfo('Received result')

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        rospy.init_node('bellbot_test_client')
        if len(sys.argv) != 2:
                rospy.loginfo('Please provide excactly one waypoint name (not %i)', len(sys.argv)-1)
                sys.exit()

        target = sys.argv[1]
        rospy.loginfo('Starting at %s', target)
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        result = bellbot_test_client(target)
        print "Result: ", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

