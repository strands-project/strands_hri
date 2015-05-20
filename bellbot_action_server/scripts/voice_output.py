#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 12:08:14 2015

@author: cdondrup
"""

import rospy
import rosparam
from actionlib import SimpleActionClient
from mary_tts.msg import maryttsAction, maryttsGoal
from bellbot_action_server.msg import BellbotState


class Manager(object):
    START = "start"
    END   = "end"

    def __init__(self, name):
        rospy.loginfo("Starting node: %s" % name)
        config_file = rospy.get_param("~config_file", '')
        if config_file == '':
            rospy.logfatal("No config file specified")
            raise rospy.ROSException("No config file specified")
            return
        self.config = rosparam.load_file(config_file)[0][0]
        print self.config
        self.last_state = ""

        rospy.loginfo("Creating mary client")
        self.mary_client = SimpleActionClient("speak", maryttsAction)
        self.mary_client.wait_for_server()
        rospy.loginfo(" ... done")
        rospy.Subscriber("/bellbot_state", BellbotState, callback=self.callback)

    def callback(self, msg):
        rospy.loginfo("STATE: %s" % msg.name)
        print self.last_state
        if self.last_state in self.config.keys():
            try:
                rospy.loginfo("saying: " + self.config[self.last_state][self.END])
                self.mary_client.send_goal_and_wait(maryttsGoal(text=self.config[self.last_state][self.END]))
            except KeyError:
                pass # No end message given

        if msg.name in self.config.keys():
            try:
                rospy.loginfo("saying: " + self.config[msg.name][self.START])
                self.mary_client.send_goal_and_wait(maryttsGoal(text=self.config[msg.name][self.START]))
            except KeyError:
                pass # No start message given

        self.last_state = msg.name


if __name__ == "__main__":
    rospy.init_node("bellbot_voice")
    lm = Manager(rospy.get_name())
    rospy.spin()