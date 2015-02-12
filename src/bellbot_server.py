#! /usr/bin/env python

import roslib; roslib.load_manifest('bellbot_action_server')
import rospy

import threading

import actionlib

from bellbot_action_server.msg import *

from bellbot_state_machine import BellbotStateMachine


class BellbotServer(object):
    # create messages that are used to publish feedback/result
    _feedback = bellbot_action_server.msg.bellbotFeedback()
    _result   = bellbot_action_server.msg.bellbotResult()
    
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, bellbot_action_server.msg.bellbotAction, execute_cb=self.execute_cb, auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server for the bellbot')

        self._stop = False
        rospy.on_shutdown(self.shutdown_hook)
        
    def is_preempt_requested(self):
        return self._as.is_preempt_requested()

    def shutdown_hook(self):
        self._stop = True

    def execute_cb(self, goal):

        rospy.loginfo('Received goal request: %s', goal)
        
        # helper variables
        r = rospy.Rate(1)
        self.success = True

        # create a state machine
        self.bellbot_sm = BellbotStateMachine()

        # set userdata: pass the whole goal
        self.bellbot_sm.set_sm_userdata(goal)        
        # run the bellbot state machine with or without introspection server
        # outcome = bellbot_sm.execute_sm()

        smach_thread = threading.Thread(target = self.bellbot_sm.execute_sm_with_introspection)
        smach_thread.start()

        #outcome = self.bellbot_sm.execute_sm_with_introspection()
        r.sleep()
        
        while self.bellbot_sm.get_sm().is_running() and not self.bellbot_sm.get_sm().preempt_requested():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested() or self._stop:
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.bellbot_sm.get_sm().request_preempt()
                self.success = False
                break

            r.sleep()

        if self.success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
        
        smach_thread.join()

if __name__ == '__main__':
    rospy.init_node('bellbot_action_server')
    bellbot = BellbotServer(rospy.get_name())
    rospy.spin()
    
    #r = rospy.Rate(1)
    #while not bellbot.is_preempt_requested():
    #    r.sleep()
