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

        self.client = actionlib.SimpleActionClient(
            'SetPTUState', PtuGotoAction)
        rospy.loginfo('Waiting for Pantilt action server...')
        self.client.wait_for_server(rospy.Duration(60))
        rospy.loginfo('Connected to pan tilt server...')

    def ptu_cb(self, data):
        self.pan_status = data.position[0] * 360 / 6.28

    def execute(self, userdata):
        rospy.loginfo("Entering Local Search mode...")

        if userdata.degree_to_go > 0:
            angles = [0, 30, -30, 0]
        else:
            angles = [0, -30, 30, 0]

        goal = PtuGotoGoal()
        for angle in angles:
            goal.pan = angle + self.pan_status
            goal.tilt = 0
            goal.pan_vel = 50
            goal.tilt_vel = 50
            self.client.send_goal(goal)
            self.client.wait_for_result()

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("Local Searching is preempted!")
