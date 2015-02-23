#! /usr/bin/env python

import sys
import os
import roslib
import rospy

import strands_webserver.client_utils

if __name__ == '__main__':
    rospy.init_node("bellbot_gui_livescreen")
    # The display to publish on, defaulting to all displays
    display_no = rospy.get_param("~display", 0)

    # tell the webserver where it should look for web files to serve
    http_root = os.path.join(roslib.packages.get_pkg_dir("bellbot_gui"), "www")
    strands_webserver.client_utils.set_http_root(http_root)

    # start with a basic page pretending things are going normally
    strands_webserver.client_utils.display_relative_page(display_no, 'livescreen.html')
    rospy.spin()
