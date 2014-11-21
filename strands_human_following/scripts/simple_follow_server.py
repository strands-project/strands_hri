#! /usr/bin/env python

import rospy
import actionlib
import threading
from strands_human_following.msg import SimpleFollowAction
from strands_human_following.human_follow import SimpleFollow


class SimpleFollowServer(object):

    def __init__(self):
        self.agent = SimpleFollow()
        self.server = actionlib.SimpleActionServer('simple_follow',
                                                   SimpleFollowAction,
                                                   self.execute, False)
        rospy.loginfo("Starting Simple Follow Server...")
        self.server.start()

    def execute(self, goal):
        rospy.loginfo('Receiving Simple Follow goal...')
        timestart = rospy.get_time()
        self.agent.sm.set_initial_state(['start'])
        self.agent.recall_preempt()

        self.smach_thread = threading.Thread(target=self.agent.execute_sm)
        self.smach_thread.setDaemon = True
        self.smach_thread.start()

        while True:
            dtime = rospy.get_time() - timestart
            if dtime > goal.time:
                self.server.set_succeeded()
                self.agent.request_preempt()
                break
            elif self.server.is_preempt_requested():
                self.server.set_preempted()
                self.agent.request_preempt()
                break

        while self.smach_thread.isAlive():
            try:
                self.smach_thread._Thread_stop()
            except:
                rospy.sleep(rospy.Duration(0.1))

        rospy.loginfo('Ending Simple Follow goal...')


if __name__ == '__main__':
    rospy.init_node('strands_human_following_server')
    server = SimpleFollowServer()
    rospy.spin()
