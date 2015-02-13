#! /usr/bin/env python

import sys
import os
import roslib
import rospy

import strands_webserver.page_utils
import strands_webserver.client_utils
import std_srvs.srv
from std_msgs.msg import String

import random
from destination_data import Destination_Data
from bellbot_action_server.srv import *
from bellbot_action_server.msg import *
from topological_utils.srv import NodeMetadata, NodeMetadataRequest, NodeMetadataResponse
from bellbot_gui.srv import *


def dummy_data():
    dests = {}
    for i in range(1, 10):
        name = "WayPoint" + str(i)
        dests[name] = Destination_Data(name=name, description=name, kind="office", goto=name, available=True)
#    for i in range(4):
#        name = "Meeting_Room_" + str(i)
#        dests[name] = Destination_Data(name=name, description=name, kind="meeting_room", goto="Lift_0", available=True)
    return dests

def random_page(choices=('http://strands-project.eu',
                             'http://www.nachrichten.at',
                             'http://www.hausderbarmherzigkeit.at')):
    return random.choice(choices)

class Bellbot_GUI(object):
    def __init__(self):
        # display a start-up page
        strands_webserver.client_utils.display_url(display_no, random_page())

        # tell the webserver where it should look for web files to serve
        self.http_root = os.path.join(roslib.packages.get_pkg_dir("bellbot_gui"), "www")
        strands_webserver.client_utils.set_http_root(self.http_root)

        self.gui_setup = GUI_Setup()
        self.gui_dest_selection = GUI_Destination_Selection()
        self.gui_operation_feedback = GUI_Operation_Feedback()
        self.gui_evaluation = GUI_User_Evaluation()
        self.gui_confirm_single_guest = GUI_Wait_For_Single_Guest()
        self.gui_confirm_multi_guests = GUI_Wait_For_Multi_Guests()

        self.states_cbs = {'Setup': self.gui_setup.display,
                           'WaitingForGoal': self.gui_dest_selection.display,
                           'WaitingForSingleGuest': self.gui_confirm_single_guest.display,
                           'WaitingForMultipleGuests':self.gui_confirm_multi_guests.display,
                           'Guiding': self.gui_operation_feedback.display,
                           'WaitingForFeedback': self.gui_evaluation.display}
        rospy.Subscriber("/bellbot_state", BellbotState, self.manage)

    def manage(self, state):
        print state
        try:
            self.states_cbs[state.name](state)
        except KeyError:
            rospy.logerr("Bellbot_GUI/manager: sad panda")
            strands_webserver.client_utils.display_url(display_no, random_page())

    def cb_feedback_finished(self, req):
        print req

class GUI_Wait_For_Single_Guest(object):
    def __init__(self):
        self.service_prefix = '/bellbot_gui_services'
        self.buttons = [('Go', 'trigger_confirm_single_guest')]
        rospy.Service(self.service_prefix+'/trigger_confirm_single_guest', std_srvs.srv.Empty, self.trigger_go)

    def display(self, state=None):
        buttons_content = strands_webserver.page_utils.generate_alert_button_page("", self.buttons, self.service_prefix)
        message = "<div>Click to confirm that you would like to go to " + state.goal + "</div>" + "<div>" + state.text + "</div>"
        www_content = message + buttons_content
        strands_webserver.client_utils.display_content(display_no, www_content)

    def trigger_go(self, req):
        rospy.wait_for_service('/bellbot_accept_target_single')
        try:
            proxy = rospy.ServiceProxy('/bellbot_accept_target_single', NewTarget)
            res = proxy(NewTargetRequest(""))
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


class GUI_Wait_For_Multi_Guests(object):
    def __init__(self):
        self.service_prefix = '/bellbot_gui_services'
        self.buttons = [('Go', 'trigger_confirm_multi_guests')]
        rospy.Service(self.service_prefix+'/trigger_confirm_multi_guests', std_srvs.srv.Empty, self.trigger_go)

    def display(self, state=None):
        buttons_content = strands_webserver.page_utils.generate_alert_button_page("", self.buttons, self.service_prefix)
        message = "<div>Click to confirm that you would like to go to " + state.goal + "</div>" + "<div>" + state.text + "</div>"
        www_content = message + buttons_content
        strands_webserver.client_utils.display_content(display_no, www_content)

    def trigger_go(self, req):
        rospy.wait_for_service('/bellbot_accept_target_mult')
        try:
            proxy = rospy.ServiceProxy('/bellbot_accept_target_mult', NewTarget)
            res = proxy(NewTargetRequest(""))
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


class GUI_Setup(object):
    def __init__(self):
        pass

    def display(self, state=None):
        strands_webserver.client_utils.display_url(display_no, random_page())


class GUI_Operation_Feedback(object):
    def __init__(self):
        pass

    def display(self, state):
        #strands_webserver.client_utils.display_relative_page(display_no, 'cake.html')
	    strands_webserver.client_utils.display_relative_page(display_no, 'livescreen.html')

class GUI_User_Evaluation(object):
    def __init__(self):
        rospy.Service('/bellbot/gui/feedback_done', FeedbackDone, self.handle_feedback_done)

    def handle_feedback_done(self, req):
        print "feedback done"
        rospy.wait_for_service('/bellbot_feedback')
        try:
            proxy = rospy.ServiceProxy('/bellbot_feedback', Feedback)
            proxy(FeedbackRequest())
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return

    def display(self, state=None):
        strands_webserver.client_utils.display_relative_page(display_no, 'feedback.html')


class GUI_Destination_Selection(object):
    def __init__(self):
        self.dests = self.get_metadata()
        self.dests_ab_sorted = sorted(self.dests.keys())
        self.service_prefix = '/bellbot_gui_services'
        self.all_buttons = []
        self.callbacks = {}
        self.cat_select_callbacks = {}
        self.dests_types = {'office': 'Offices', 'Meeting Rooms': 'Meeting Rooms'}

        name = "Click to select destination type:"
        cat_select_buttons = [('Offices', "select_offices"), ("Meeting Rooms", "select_meeting_rooms")]
        self.cat_select_content = strands_webserver.page_utils.generate_alert_button_page(name, cat_select_buttons, self.service_prefix)
        rospy.Service(self.service_prefix + "/select_offices", std_srvs.srv.Empty, self.cb_offices)
        rospy.Service(self.service_prefix + "/select_meeting_rooms", std_srvs.srv.Empty, self.cb_meeting_rooms)

        for k in self.dests_ab_sorted:
            srv_name = self.service_prefix + "/select_" + self.dests[k].id
            self.callbacks[k] = Callback_Trigger_Select_Destination(self.dests[k], self)
            rospy.Service(srv_name, std_srvs.srv.Empty, self.callbacks[k].trigger)
            self.all_buttons.append((self.dests[k].name, "select_" + self.dests[k].id))
        name = 'Click to select your destination:'
        self.www_content = strands_webserver.page_utils.generate_alert_button_page(name, self.all_buttons, self.service_prefix)

    def display(self, state=None):
        # strands_webserver.client_utils.display_content(display_no, self.www_content)
        strands_webserver.client_utils.display_content(display_no, self.cat_select_content)

    def cb_offices(self, req):
        buttons = []
        for k in self.dests_ab_sorted:
            if self.dests[k].kind == 'office':
                buttons.append((self.dests[k].name, "select_" + self.dests[k].id))
        name = 'Click to select your destination:'
        content = strands_webserver.page_utils.generate_alert_button_page(name, buttons, self.service_prefix)
        strands_webserver.client_utils.display_content(display_no, content)

    def cb_meeting_rooms(self, req):
        buttons = []
        for k in self.dests_ab_sorted:
            if self.dests[k].kind == 'Meeting Rooms':
                buttons.append((self.dests[k].name, "select_" + self.dests[k].id))
        name = 'Click to select your destination:'
        content = strands_webserver.page_utils.generate_alert_button_page(name, buttons, self.service_prefix)
        strands_webserver.client_utils.display_content(display_no, content)

    def get_metadata(self):
        dests = {}
        rospy.wait_for_service('/query_node_metadata')
        try:
            print "Getting offices"
            proxy = rospy.ServiceProxy('/query_node_metadata', NodeMetadata)
            res = proxy(NodeMetadataRequest("lg_20140618", "lg_20140618", 'office')) # 'office' | 'Meeting Rooms'
            for i in range(0, len(res.name)):
                dests[res.name[i]] = Destination_Data(name=res.name[i], description=res.description[i], kind=res.node_type[i], goto=res.goto_node[i], available=True, at_node=res.at_node[i])

            print "Getting meeting rooms"
            res = proxy(NodeMetadataRequest("lg_20140618", "lg_20140618", 'Meeting Rooms')) # 'office' | 'Meeting Rooms'

            for i in range(0, len(res.name)):
                dests[res.name[i]] = Destination_Data(name=res.name[i], description=res.description[i], kind=res.node_type[i], goto=res.goto_node[i], available=True, at_node=res.at_node[i])
            return dests

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        #return dummy_data()


class Callback_Trigger_Select_Destination(object):
    def __init__(self, dest, mama):
        self.dest = dest
        self.mama = mama
        self.service_prefix = '/bellbot_gui_services'

        self.buttons = [('Zuruck doch', 'trigger_back_'+self.dest.id), ('Geh ma', 'trigger_go_'+self.dest.id)]
        self.www_content = strands_webserver.page_utils.generate_alert_button_page("__REPLACE_ME__", self.buttons, self.service_prefix)
        self.extra_content()

        rospy.Service(self.service_prefix+'/trigger_go_'+self.dest.id, std_srvs.srv.Empty, self.trigger_go)
        rospy.Service(self.service_prefix+'/trigger_back_'+self.dest.id, std_srvs.srv.Empty, self.trigger_back)

    def extra_content(self):
        if self.dest.at_node:
          new = '<div><h3 color="red">' + 'I am about to guide you to ' + self.dest.name + '. Click to go now or return to main menu.' + '</h3></div>'
        else:
          new = '<div><h3><font color="red">Warning: </font>' + 'I am afraid I can only guide you upto a point. See below for further directions. Click to go now or return to main menu.'  + '</h3></div>'
        old = '<div class="notice">__REPLACE_ME__</div>'
        self.www_content = self.www_content.replace(old, new)

        foo = "<hr>"
        foo += "<div><h4>Descrition:</h4>"
        foo += self.dest.description
        foo += "</div>"
        self.www_content += foo

    def trigger(self, req):
        print "button " + self.dest.name + " pressed."
        strands_webserver.client_utils.display_content(display_no, self.www_content)

    def trigger_go(self, req):
        print "go pressed"
        rospy.wait_for_service('/bellbot_new_target')
        try:
          proxy = rospy.ServiceProxy('/bellbot_new_target', NewTarget)
          res = proxy(NewTargetRequest(self.dest.goto))
          return res
          #return NewTargetResponse()
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e


    def trigger_back(self, req):
        print "back pressed"
        self.mama.display()



if __name__ == '__main__':
    rospy.init_node("bellbot_gui")
    # The display to publish on, defaulting to all displays
    display_no = rospy.get_param("~display", 0)

    gui = Bellbot_GUI()
    rospy.spin()
