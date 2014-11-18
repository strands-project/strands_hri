#! /usr/bin/env python

import rospy
import actionlib
from strands_human_following.msg import SimpleFollowAction, SimpleFollowGoal


class SimpleFollowClient(object):

    def __init__(self):
        rospy.on_shutdown(self.shutdown)

        self.client = actionlib.SimpleActionClient('simple_follow',
                                                   SimpleFollowAction)
        rospy.loginfo("Waiting for server...")
        self.client.wait_for_server()

        rospy.loginfo("Sending a goal...")
        self.client.send_goal(SimpleFollowGoal(time=3000))

    def shutdown(self):
        self.client.cancel_all_goals()


if __name__ == '__main__':
    rospy.init_node('strands_human_following_client', anonymous=False)
    try:
        client = SimpleFollowClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Program was interrupted before completion!")
