#! /usr/bin/env python
import rospy
import smach
import smach_ros
import sys
import getopt
import random
import json

from agent import Agent

import actionlib
from actionlib_msgs.msg import *
import topological_navigation.msg


class BellbotStateMachine(Agent):

    """
    Definition of the bellbot state machine. 

    """
    def __init__(self):

        # the super().__init__() methods calls make_sm(),
        super(BellbotStateMachine,self).__init__()
       
    
    def make_sm(self):
        self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        
        with self.sm:
            
            smach.StateMachine.add('Setup', Setup(), 
                                   transitions={ 'succeeded': 'WaitingForGoal', 
                                                 'aborted': 'aborted',
                                                 'preempted': 'preempted' 
                                               })
            smach.StateMachine.add('WaitingForGoal', WaitingForGoal(), 
                                   transitions={ 'succeeded': 'Guiding', 
                                                 'aborted': 'aborted',
                                                 'preempted': 'preempted' 
                                               })
            smach.StateMachine.add('Guiding', Guiding(), 
                                   transitions={ 'succeeded': 'WaitingForFeedback', 
                                                 'aborted': 'aborted',
                                                 'preempted': 'preempted' 
                                               })
            smach.StateMachine.add('WaitingForFeedback', WaitingForFeedback(), 
                                   transitions={ 'succeeded': 'Setup', 
                                                 'aborted': 'aborted',
                                                 'preempted': 'preempted' 
                                               })
            return self.sm

    def set_sm_userdata(self, goal):
        """Set userdata for state machine."""
        rospy.loginfo('Getting parameters from server')

        self.sm.userdata.goal  = goal 

        
    def get_sm_userdata(self):
        return self.sm.userdata

#______________________________________________________________________________
# behavior

class Setup(smach.State):

    """
    Move the robot to a requested location to provide the Bellbot service.
    """

    def __init__(self):
        smach.State.__init__(self,
                           outcomes=['succeeded', 'aborted', 'preempted'],
                           input_keys=['goal'],
                           output_keys=[])


        rospy.on_shutdown(self._on_node_shutdown)
        
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        
        self.client.wait_for_server()
        
        rospy.loginfo(" Setup ... Init done")
        
    def _on_node_shutdown(self):
        self.client.cancel_all_goals()

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        navgoal = topological_navigation.msg.GotoNodeGoal()
        
        rospy.loginfo("Requesting Navigation to %s", userdata.goal.starting_waypoint_name)
        navgoal.target = userdata.goal.starting_waypoint_name
        
        # Sends the goal to the action server.
        self.client.send_goal(navgoal)

        # Waits for the server to finish performing the action.
        status=self.client.get_state()
        while (status == GoalStatus.ACTIVE or status == GoalStatus.PENDING):
            if self.preempt_requested(): 
                self.service_preempt()
                self.client.cancel_all_goals()
                return 'preempted'
            else:
                status=self.client.get_state()
                rospy.sleep(rospy.Duration.from_sec(0.01))
        
        # Check if the naviation was successful 
        status = self.client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded' 
        else:
            return 'aborted'

class WaitingForGoal(smach.State):

    """
    Show the GUI and wait for input.
    """

    def __init__(self):
        smach.State.__init__(self,
                           outcomes=['succeeded', 'aborted', 'preempted'],
                           input_keys=[],
                           output_keys=['target'])

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        userdata.target = 'WayPoint3'

        return 'succeeded' 

class Guiding(smach.State):

    """
    Guide a person to the required destination.
    """

    def __init__(self):
        smach.State.__init__(self,
                           outcomes=['succeeded', 'aborted', 'preempted'],
                           input_keys=['target'],
                           output_keys=[])

        rospy.on_shutdown(self._on_node_shutdown)
        
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        
        self.client.wait_for_server()
        
        rospy.loginfo('Guiding ... Init done')
        
    def _on_node_shutdown(self):
        self.client.cancel_all_goals()

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        navgoal = topological_navigation.msg.GotoNodeGoal()
        
        rospy.loginfo("Requesting Navigation to %s", userdata.target)
        navgoal.target = userdata.target
        
        # Sends the goal to the action server.
        self.client.send_goal(navgoal)

        # Waits for the server to finish performing the action.
        status=self.client.get_state()
        while (status == GoalStatus.ACTIVE or status == GoalStatus.PENDING):
            if self.preempt_requested(): 
                self.service_preempt()
                self.client.cancel_all_goals()
                return 'preempted'
            else:
                status=self.client.get_state()
                rospy.sleep(rospy.Duration.from_sec(0.01))
        
        # Check if the naviation was successful 
        status  = self.client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded' 
        else:
            return 'aborted'

class WaitingForFeedback(smach.State):

    """
    Wait for feedback on this bellbot tour. 
    """

    def __init__(self):
        smach.State.__init__(self,
                           outcomes=['succeeded', 'aborted', 'preempted'],
                           input_keys=[],
                           output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        return 'succeeded' 


