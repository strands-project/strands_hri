#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import roslib
import actionlib
#import actionlib_msgs.msg
import strands_interaction_behaviours.msg
import ros_mary_tts.msg
import std_srvs.srv
#import memory_game_msgs.msg
import strands_webserver.page_utils
import strands_webserver.client_utils
#import strands_hri_utils.msg

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

        # Lights client
        #rospy.loginfo("%s: Creating lights client", name)
        #self.lightsClient = actionlib.SimpleActionClient('visual_speech', strands_hri_utils.msg.VisualSpeechAction)
        #self.lightsClient.wait_for_server()
        #rospy.loginfo("%s: ...done", name)

        # Executor client
        #rospy.loginfo("%s: Creating executor client", name)
        #self.exeClient = actionlib.SimpleActionClient('memory_game_executor',memory_game_msgs.msg.PlayMemoryGameAction)
        #self.exeClient.wait_for_server()
        #rospy.loginfo("%s: ...done", name)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(self._action_name, strands_interaction_behaviours.msg.InteractionEngagedAction, execute_cb= None, auto_start = False)
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # Services for buttons
        #rospy.Service(name+'/schedule_game', std_srvs.srv.Empty, self.scheduleGame)
        rospy.Service(name+'/show_info', std_srvs.srv.Empty, self.showInfo)

        # tell the webserver where it should look for web files to serve
        #http_root = os.path.join(roslib.packages.get_pkg_dir("strands_webserver"), "data")
        #strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('y1_interfaces') + '/www')
        #strands_webserver.client_utils.set_http_root(http_root)


    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        rospy.logdebug("Received goal:\n%s",self._goal)
        goal = ros_mary_tts.msg.maryttsGoal()
        goal.text = "Hallo, Ich bin der Henry!"
        self.maryClient.send_goal(goal)
        goal = ros_mary_tts.msg.maryttsGoal()
        goal.text = "Möchtest du mehr über mich erfahren?" # oder mit mir spielen?"
        self.maryClient.send_goal(goal)
        # Build pagec
        name = '<div id="logo-left" style="height:500px;width:300px;float:left;"><img src="strands-logo.png" width=300px"></div><center><p><b>Was möchtest du tun?</b></p></centre><div id="footer" style="text-align:center;font-size:75%;"><img src="aaf-logo.png" style="float:center"></div>'
        #buttons = [('Ein Spiel spielen', 'schedule_game'), ('Erzähl mir von dir', 'show_info')]
        buttons = [('Erzähl mir von dir', 'show_info')]
        service_prefix = self._action_name
        content = strands_webserver.page_utils.generate_alert_button_page(name, buttons, service_prefix)
        strands_webserver.client_utils.display_content(self.display_no, content)

    def preemptCallback(self):
        rospy.loginfo("Action is being preempted")
        self.maryClient.cancel_all_goals()
        #self.exeClient.cancel_all_goals()
        self._as.set_preempted()

    #def scheduleGame(self, req):
        #goal = memory_game_msgs.msg.PlayMemoryGameGoal()
        #self.exeClient.send_goal_and_wait(goal)
        #if self.exeClient.get_result() == actionlib_msgs.msg.GoalStatus.SUCCEEDED and not self.exeClient.is_preempting():
            #self._as.set_succeeded()
        #else:
            #self._as.set_abborted()

    def showInfo(self, req):
        page = 'strands-aaf-info1.html'
        strands_webserver.client_utils.display_relative_page(self.display_no, page)
        sentence = {"Ich bin Henry, der Roboter",
                    "Ich werde in einem EU-Forschungsprojekt entwickelt.",
                    "Ziel ist es, für Sicherheit und Unterstützung im Arbeitsalltag zu sorgen.",
                    "Dafür werde ich im Haus der Barmherzigkeit getestet."
                    }
        goal = ros_mary_tts.msg.maryttsGoal()
        for x in sentence:
            if self._as.is_active():
                goal.text = x
                self.maryClient.send_goal_and_wait(goal)
        #rospy.sleep(rospy.Duration(240))
        #self._as.set_preempted()

	#self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('engaged_server')
    EngagedServer(rospy.get_name())
    rospy.spin()

