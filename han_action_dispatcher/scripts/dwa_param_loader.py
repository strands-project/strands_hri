#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam


class ParamLoader(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s" % name)
        a_list = rospy.get_param("han_action_dispatcher")["han_actions"]
        param_file = rospy.get_param("~param_file")
        for a in a_list.keys():
            rosparam.load_file(param_file, a, True)
        rospy.loginfo("done")
        
        
if __name__ == "__main__":
    rospy.init_node("dwa_param_loader")
    ParamLoader(rospy.get_name())
    