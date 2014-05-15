#! /usr/bin/env python

import rospy
import actionlib
#import actionlib_msgs.msg
import strands_gazing.msg
from scitos_apps_msgs.msg import action_buttons

class GazeSwitch(object):
    # create messages that are used to publish feedback/result

    def __init__(self, name):
        # Variables
        self.remote = True

        #Getting parameters
        self.topic = rospy.get_param("~topic", "/upper_body_detector/closest_bounding_box_centre")

        # Gaze client
        rospy.loginfo("%s: Creating gaze client", name)
        self.gazeClient = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)
        self.gazeClient.wait_for_server()
        self.goal = strands_gazing.msg.GazeAtPoseGoal()
        self.goal.topic_name = self.topic
        rospy.loginfo("%s: ...done", name)

        rospy.Subscriber("/teleop_joystick/action_buttons", action_buttons, self.button_callback)

    def button_callback(self, action_buttons):
        if(action_buttons.A):
            if self.gazeClient.simple_state == actionlib.SimpleGoalState.ACTIVE:
                self.gazeClient.cancel_all_goals()
            else:
                self.gazeClient.send_goal(self.goal)


if __name__ == '__main__':
    rospy.init_node('gaze_switch')
    GazeSwitch(rospy.get_name())
    rospy.spin()

