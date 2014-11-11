#! /usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
from scitos_ptu.msg import PtuGotoGoal, PtuGotoAction
from strands_human_following.wander_search import Wander, Search
from strands_human_following.follow_and_move import Follow, MoveSearch
from strands_human_following.local_search import LocalSearch


class SimpleFollow(object):

    def __init__(self):
        rospy.loginfo("Starting simple follow state machine...")
        self.sm = smach.StateMachine(outcomes=['succeeded',
                                               'aborted',
                                               'preempted'])

    def build_sm(self):
        with self.sm:
            ptu_goal = PtuGotoGoal()
            ptu_goal.pan = 0
            ptu_goal.tilt = 0
            ptu_goal.pan_vel = 100
            ptu_goal.tilt_vel = 100

            smach.StateMachine.add('start', SimpleActionState(
                'SetPTUState', PtuGotoAction, goal=ptu_goal),
                transitions={'succeeded': 'wander_search'})

            wander_search = smach.Concurrence(
                outcomes=['succeeded', 'aborted', 'preempted'],
                default_outcome='succeeded',
                child_termination_cb=self.child_termination_cb,
                outcome_cb=self.outcome_cb)

            with wander_search:
                smach.Concurrence.add('Wandering', Wander())
                smach.Concurrence.add('Searching', Search())
            smach.StateMachine.add('wander_search', wander_search,
                                   transitions={
                                       'succeeded': 'wander_search',
                                       'aborted': 'follow'})

            follow = smach.Concurrence(
                outcomes=['succeeded', 'aborted', 'preempted'],
                default_outcome='aborted',
                output_keys=['degree_to_go'],
                outcome_map={'succeeded': {'MoveSearch': 'succeeded'}})

            with follow:
                smach.Concurrence.add('Follow', Follow())
                smach.Concurrence.add('MoveSearch', MoveSearch())
            smach.StateMachine.add('follow', follow,
                                   transitions={'succeeded': 'follow',
                                                'aborted': 'local_search'})

            local_search = smach.Concurrence(
                outcomes=['succeeded', 'aborted', 'preempted'],
                default_outcome='aborted',
                input_keys=['degree_to_go'],
                outcome_map={'succeeded': {'MoveSearch': 'succeeded'}})

            with local_search:
                smach.Concurrence.add('LocalSearch', LocalSearch())
                smach.Concurrence.add('MoveSearch', MoveSearch())
            smach.StateMachine.add('local_search', local_search,
                                   transitions={'succeeded': 'follow',
                                                'aborted': 'start'})

    def child_termination_cb(self, msg):
        if msg['Searching'] == 'succeeded':
            return True
        else:
            rospy.loginfo("Child termination failed!")
            return False

    def outcome_cb(self, msg):
        if msg['Searching'] == 'succeeded':
            return 'aborted'
        else:
            return 'succeeded'

    def execute_sm(self):
        self.build_sm()
        self.sm.execute()


if __name__ == '__main__':
    rospy.init_node('strands_human_following_sm')
    sm = SimpleFollow()
    rospy.spin()
