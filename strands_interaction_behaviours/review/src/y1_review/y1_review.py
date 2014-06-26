#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import strands_interaction_behaviours.msg
import actionlib_msgs.msg
import std_msgs.msg
import strands_perception_people_msgs.msg


class YoneReview(object):
    # create messages that are used to publish feedback/result
    #_feedback = strands_interaction_behaviours.msg.BehaviourSwitchFeedback()

    def __init__(self, name):
        # Variables
        self._action_name = name

        #Getting parameters

        # BehaviourSwitch client
        rospy.loginfo("Creating idle_behaviour client")
        self.ibClient = actionlib.SimpleActionClient(
            'idle_behaviour',
            strands_interaction_behaviours.msg.IdleBehaviourAction
        )
        self.ibClient.wait_for_server()
        rospy.loginfo("...done")

        # Waiting for services
        rospy.loginfo("Waiting for services")
        rospy.wait_for_service('/start_head_analysis')
        rospy.wait_for_service('/stop_head_analysis')
        rospy.loginfo("...done")

        # Starting server
        rospy.loginfo("Starting action server")
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            strands_interaction_behaviours.msg.ReviewAction,
            execute_cb=self.exCallback,
            auto_start=False
        )
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("...done.")

    def exCallback(self, goal):
        try:
            s = rospy.ServiceProxy(
                    '/start_head_analysis',
                    strands_perception_people_msgs.msg.StartHeadAnalysis
                    )
            s()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        goal = strands_interaction_behaviours.msg.IdleBehaviourGoal()
        ibClient.send_goal_and_wait(goal)
        try:
            s = rospy.ServiceProxy(
                    '/stop_head_analysis',
                    strands_perception_people_msgs.msg.StopHeadAnalysis
                    )
            s()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        if self.ibClient.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            self._as.set_succeeded()
        elif not self._as.is_preempt_requested():
            self._as.set_aborted()

    def preemptCallback(self):
        self.ibClient.cancel_all_goals()
        try:
            s = rospy.ServiceProxy(
                    '/stop_head_analysis',
                    strands_perception_people_msgs.msg.StopHeadAnalysis
                    )
            s()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        self._as.set_preempted()


if __name__ == '__main__':
    rospy.init_node('y1_review')
    YoneReview(rospy.get_name())
    rospy.spin()
