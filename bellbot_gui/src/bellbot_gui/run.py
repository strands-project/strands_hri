#! /usr/bin/env python

import rospy
from gui import GUI_Destination_Selection
import strands_webserver.page_utils
import strands_webserver.client_utils
import std_srvs.srv

if __name__ == '__main__':
    rospy.init_node("bellbot_gui")
    # The display to publish on, defaulting to all displays
    display_no = rospy.get_param("~display", 0)

    gui = GUI_Destination_Selection()
    gui.display()
    rospy.spin()
