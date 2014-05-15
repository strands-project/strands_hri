#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import strands_interaction_behaviours.msg
import actionlib_msgs.msg
import flir_pantilt_d46.msg
import std_srvs.srv
#import y1_interfaces.page_utils
import std_msgs.msg
import strands_webserver.client_utils


class IdleBehaviour(object):
# create messages that are used to publish feedback/result
    _feedback = strands_interaction_behaviours.msg.BehaviourSwitchFeedback()

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.mode = 0

        #Getting parameters
        self.runtime = rospy.get_param("~runtime", 0)
        self.display_no = rospy.get_param("~display", 0)
        engage_topic = rospy.get_param("~engage_topic", '/engagement_checker/engaged')

        #Publishers
        self.pub = rospy.Publisher(engage_topic,std_msgs.msg.Bool,queue_size=100)

        # BehaviourSwitch client
        rospy.loginfo("%s: Creating behaviour_switch client", name)
        self.bsClient = actionlib.SimpleActionClient('behaviour_switch', strands_interaction_behaviours.msg.BehaviourSwitchAction)
        self.bsClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # PTU client
        rospy.loginfo("%s: Creating PTU client", name)
        self.ptuClient = actionlib.SimpleActionClient('SetPTUState', flir_pantilt_d46.msg.PtuGotoAction)
        self.ptuClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(self._action_name, strands_interaction_behaviours.msg.IdleBehaviourAction, execute_cb=self.exCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # Services for buttons
        rospy.Service(name+'/engage', std_srvs.srv.Empty, self.engage)

    def exCallback(self, goal):
        #self.createPage()
        rospy.loginfo("Created page")
    	self.turnPTU(-180)
        rospy.loginfo("Turned PTU")
        idle_goal = strands_interaction_behaviours.msg.BehaviourSwitchGoal()
        idle_goal.runtime_seconds = self.runtime
        self.bsClient.send_goal(idle_goal)
        self.bsClient.wait_for_result()
        self.turnPTU(0)
        #self.createPage()
        if self.bsClient.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            self._as.set_succeeded()
        elif not self._as.is_preempt_requested():
            self._as.set_aborted()

	#Setting http root
	#strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('y1_interfaces') + '/www')

    def preemptCallback(self):
        self.bsClient.cancel_all_goals()
        strands_webserver.client_utils.display_relative_page(self.display_no, 'index.html')
        self._as.set_preempted()

    def turnPTU(self, pan):
        goal = flir_pantilt_d46.msg.PtuGotoGoal()
        goal.pan = pan
        goal.tilt = 0
        goal.pan_vel = 60
        goal.tilt_vel = 0
        self.ptuClient.send_goal_and_wait(goal)

    def engage(self,req):
        rospy.loginfo("Engage button pressed")
        a = std_msgs.msg.Bool()
        a.data = True
        self.pub.publish(a)

if __name__ == '__main__':
    rospy.init_node('idle_behaviour')
    IdleBehaviour(rospy.get_name())
    rospy.spin()

