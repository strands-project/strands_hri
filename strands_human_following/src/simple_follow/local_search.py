#! /usr/bin/env python

import rospy
import smach
import actionlib
from scitos_ptu.msg import PtuGotoAction, PtuGotoGoal
from sensor_msgs.msg import JointState


class LocalSearch(smach.State):

    def __init__(self):
        rospy.loginfo('Entering local search state...')
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['degree_to_go'])

        rospy.Subscriber('/ptu/states', JointState, self.ptu_cb)
        self.pan_status = 0.0

    def ptu_cb(self, data):
        self.pan_status = data.position[0] * 360 / 6.28

    def execute(self, userdata):
        rospy.sleep(rospy.Duration(0.5))
        if userdata.degree_to_go > 0:
            angles = [0, 30, -30, 0]
        else:
            angles = [0, -30, 30, 0]

        client = actionlib.SimpleActionClient(
            'SetPTUState', PtuGotoAction)
        rospy.loginfo('Waiting for Pantilt action server...')
        client.wait_for_server(rospy.Duration(60))
        rospy.loginfo('Connected to pan tilt server...')

        goal = PtuGotoGoal()

        for angle in angles:
            goal.pan = angle + self.pan_status
            goal.tilt = 0
            goal.pan_vel = 100
            goal.tilt_vel = 50

            if self.preempt_requested():
                return 'succeeded'

            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(1))
            rospy.sleep(rospy.Duration(0.5))

        return 'succeeded'
