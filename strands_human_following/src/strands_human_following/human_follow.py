#! /usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
from scitos_ptu.msg import PtuGotoGoal, PtuGotoAction
from strands_human_following.wander_search import Wander
from strands_human_following.follow_and_move import Follow, MoveSearch
from strands_human_following.local_search import LocalSearch


class SimpleFollow(object):

    def __init__(self):
        rospy.loginfo("Starting simple follow state machine...")
        self.sm = smach.StateMachine(outcomes=['succeeded',
                                               'aborted',
                                               'preempted'])

        with self.sm:
            ptu_goal = PtuGotoGoal()
            ptu_goal.pan = 0
            ptu_goal.tilt = 0
            ptu_goal.pan_vel = 100
            ptu_goal.tilt_vel = 100

            smach.StateMachine.add('start', SimpleActionState(
                'SetPTUState', PtuGotoAction, goal=ptu_goal),
                transitions={'succeeded': 'wander_search',
                             'preempted': 'preempted'})

            smach.StateMachine.add(
                'wander_search', Wander(),
                transitions={'succeeded': 'follow',
                             'preempted': 'preempted',
                             'aborted': 'wander_search'})

            self.follow = smach.Concurrence(
                outcomes=['succeeded', 'aborted', 'preempted'],
                default_outcome='aborted',
                output_keys=['degree_to_go'],
                # input_keys=['current_uuid', 'current_robot',
                #     'current_pose_tf'],
                outcome_cb=self.outcome_follow)

            with self.follow:
                smach.Concurrence.add('Follow', Follow())
                smach.Concurrence.add('MoveSearch', MoveSearch())
            smach.StateMachine.add('follow', self.follow,
                                   transitions={'succeeded': 'follow',
                                                'preempted': 'preempted',
                                                'aborted': 'local_search'})

            self.local_search = smach.Concurrence(
                outcomes=['succeeded', 'aborted', 'preempted'],
                default_outcome='aborted',
                # input_keys=['degree_to_go', 'current_uuid',
                #             'current_robot', 'current_pose_tf'],
                input_keys=['degree_to_go'],
                outcome_cb=self.outcome_local)

            with self.local_search:
                smach.Concurrence.add('LocalSearch', LocalSearch())
                smach.Concurrence.add('MoveSearch', MoveSearch())
            smach.StateMachine.add('local_search', self.local_search,
                                   transitions={'succeeded': 'follow',
                                                'preempted': 'preempted',
                                                'aborted': 'start'})

    def outcome_follow(self, msg):
        if msg['Follow'] == 'preempted' or msg['MoveSearch'] == 'preempted':
            return 'preempted'
        elif msg['MoveSearch'] == 'succeeded':
            return 'succeeded'
        else:
            return 'aborted'

    def outcome_local(self, msg):
        if msg['LocalSearch'] == 'preempted' or msg['MoveSearch'] == 'preempted':
            return 'preempted'
        elif msg['MoveSearch'] == 'succeeded':
            return 'succeeded'
        else:
            return 'aborted'

    def execute_sm(self):
        self.sm.execute()

    def request_preempt(self):
        rospy.loginfo("Stopping all processes...")
        self.sm.request_preempt()

    def recall_preempt(self):
        self.sm.recall_preempt()


if __name__ == '__main__':
    rospy.init_node('strands_human_following_sm')
    sm = SimpleFollow()
    rospy.spin()
