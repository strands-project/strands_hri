#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib.simple_action_server import SimpleActionServer
from actionlib.simple_action_client import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
from dynamic_reconfigure.server import Server as DynServer
from han_action_dispatcher.cfg import HanActionDispatcherConfig


class ActionDispatcher(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s" % name)
        self.client = None
        action_list = rospy.get_param("han_action_dispatcher")["han_actions"]
        self.default_action = rospy.get_param("~default_action")
        self.dyn_srv = DynServer(HanActionDispatcherConfig, self.dyn_callback)
        self.servers = {}
        for a in action_list.items():
            name = a[0]
            self.servers[name] = SimpleActionServer(
                name, 
                MoveBaseAction,
                execute_cb=(lambda x: lambda msg: self.execute_cb(msg, x[0], x[1]["action"]))(a),
                auto_start=False
            )
            self.servers[name].register_preempt_callback(self.preempt_cb)
            self.servers[name].start()
        rospy.loginfo("done")
        
    def execute_cb(self, msg, name, action):
        a = action if not self.use_default else self.default_action
        self.client = SimpleActionClient(a, MoveBaseAction)
        rospy.logdebug("Waiting for action server:" + a)
        self.client.wait_for_server()
        rospy.logdebug("Sending goal to:" + a)
        self.client.send_goal_and_wait(msg)
        self.servers[name].set_succeeded()
        self.client = None
        
    def preempt_cb(self):
        if self.client != None:
            self.client.cancel_all_goals()
            self.client = None
        
    def dyn_callback(self, config, level):
        self.use_default = config["use_default"]
        return config
        
        
if __name__ == "__main__":
    rospy.init_node("han_action_dispatcher")
    ActionDispatcher(rospy.get_name())
    rospy.spin()