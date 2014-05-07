#! /usr/bin/env python

import rospy
import roslib
import actionlib
import strands_interaction_behaviours.msg
import ros_mary_tts.msg
import std_srvs.srv
import os
import strands_webserver.page_utils
import strands_webserver.client_utils

class EngagedServer(object):
# create messages that are used to publish feedback/result
    _feedback = strands_interaction_behaviours.msg.BehaviourSwitchFeedback()

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.mode = 0

        #Getting parameters
        self.runtime = rospy.get_param(self._action_name + "/runtime", 0)
        self.display_no = rospy.get_param("~display", 0)

        # Mary client
        rospy.loginfo("%s: Creating mary client", name)
        self.maryClient = actionlib.SimpleActionClient('speak', ros_mary_tts.msg.maryttsAction)
        self.maryClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(self._action_name, strands_interaction_behaviours.msg.InteractionEngagedAction, execute_cb= None, auto_start = False)
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # Services for buttons
        rospy.Service(name+'/schedule_game', std_srvs.srv.Empty, self.scheduleGame)
        rospy.Service(name+'/show_info', std_srvs.srv.Empty, self.showInfo)

        # tell the webserver where it should look for web files to serve
        http_root = os.path.join(roslib.packages.get_pkg_dir("strands_webserver"), "data")
        strands_webserver.client_utils.set_http_root(http_root)


    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        rospy.logdebug("Received goal:\n%s",self._goal)
        goal = ros_mary_tts.msg.maryttsGoal()
        goal.text = "Engage!"
        self.maryClient.send_goal(goal)
        # Build page
        name = 'Would you like to know more?'
        buttons = [('Play a game', 'schedule_game'), ('Show more', 'show_info')]
        service_prefix = self._action_name
        content = strands_webserver.page_utils.generate_alert_button_page(name, buttons, service_prefix)
        strands_webserver.client_utils.display_content(self.display_no, content)

    def preemptCallback(self):
        self.maryClient.cancel_all_goals()
        self._as.set_preempted()

    def scheduleGame(self, req):
        rospy.loginfo("STOP AND SCHEDULE TASK")
        self._as.set_succeeded()

    def showInfo(self, req):
        rospy.loginfo("SHOW INFORMATION ABOUT ROBOT")
        self._as.set_succeeded()



if __name__ == '__main__':
    rospy.init_node('engaged_server')
    EngagedServer(rospy.get_name())
    rospy.spin()

