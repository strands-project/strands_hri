#! /usr/bin/env python
import rospy
import smach
import smach_ros

class Thing (object):
    pass

class Agent (Thing):
    """Generic definition of an agent that uses a state machine."""
    def __init__(self):
        self.sm = self.make_sm()
        
    def make_sm(self):
        """Creates and returns the agent's state machine."""
        return smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    def get_sm(self):
        """Returns the state machine"""
        return self.sm
    
    def execute_sm(self):
        """Execute the agent's state machine."""
        rospy.loginfo("start state machine...")
        return self.sm.execute()
    def execute_sm_with_introspection(self):
        """Execute the agent's state machine with introspection"""
        rospy.loginfo("start introspection server...")
        sis = smach_ros.IntrospectionServer('introspection_server', self.sm, '/SM_ROOT')
        sis.start()
        outcome = self.execute_sm()
        sis.stop()
        
    ## hack to pickle thread.Lock object     
    def __getstate__(self):
        state = self.sm.__dict__.copy()
        del state['_state_transitioning_lock']
        return state

    def __setstate__(self, state):
        self.sm.__dict__ = state
        self.sm._state_transitioning_lock = threading.Lock()
