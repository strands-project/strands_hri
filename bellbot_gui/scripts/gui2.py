#! /usr/bin/env python

import sys
import ConfigParser
import os
import subprocess
import roslib
import rospy
from datetime import datetime
import csv

import strands_webserver.page_utils
import strands_webserver.client_utils
import std_srvs.srv
from std_msgs.msg import String

import random
from bellbot_gui.destination_data import Destination_Data
from bellbot_action_server.srv import *
from bellbot_action_server.msg import *
from topological_utils.srv import NodeMetadata, NodeMetadataRequest, NodeMetadataResponse

def random_page(choices=('http://strands-project.eu',
                             'http://www.nachrichten.at',
                             'http://www.hausderbarmherzigkeit.at')):
    return random.choice(choices)

def florence():
    try:
        ids = subprocess.check_output(["pidof", "florence"])
        if len(ids.split()) > 1:
            subprocess.call(["pkill", "florence"])
            try:
                subprocess.Popen("florence")
            except:
                rospy.logerr("virtual keyboard not found, please install it by running:\nsudo apt-get install florence")
    except:
        try:
            subprocess.Popen("florence")
            # pass
        except:
            rospy.logerr("virtual keyboard not found, please install it by running:\nsudo apt-get install florence")


class Bellbot_GUI(object):
    def __init__(self):
        # display a start-up page
        strands_webserver.client_utils.display_url(display_no, random_page())

        self.deployment = rospy.get_param("/bellbot_gui/deployment")

        self.pre_setup()

        # tell the webserver where it should look for web files to serve
        # self.http_root = os.path.join(roslib.packages.get_pkg_dir("bellbot_gui"), "www")
        self.http_root = os.path.join(roslib.packages.get_pkg_dir(self.deployment), "www")
        strands_webserver.client_utils.set_http_root(self.http_root)

        self.gui_setup = GUI_Setup()
        self.gui_dest_selection = GUI_Destination_Selection(self.http_root, self.write_dir, self.deployment)
        self.gui_operation_feedback = GUI_Operation_Feedback()
        self.gui_evaluation = GUI_User_Evaluation(self.http_root, self.write_dir, self.deployment)
        # self.gui_confirm_single_guest = GUI_Wait_For_Single_Guest()
        # self.gui_confirm_multi_guests = GUI_Wait_For_Multi_Guests()

        self.states_cbs = {'Setup': self.gui_setup.display,
                           'WaitingForGoal': self.gui_dest_selection.display,
                           # 'WaitingForSingleGuest': self.gui_confirm_single_guest.display,
                           # 'WaitingForMultipleGuests':self.gui_confirm_multi_guests.display,
                           'Guiding': self.gui_operation_feedback.display,
                           'WaitingForFeedback': self.gui_evaluation.display}
        rospy.Subscriber("/bellbot_state", BellbotState, self.manage)
        rospy.on_shutdown(self._on_node_shutdown)

    def pre_setup(self):
        # writeable directory
        self.write_dir = os.path.join(os.environ["HOME"], ".ros", "bellbot")
        if not os.path.exists(self.write_dir):
            os.makedirs(self.write_dir)

        # move www to deployment package
        copy_to = roslib.packages.get_pkg_dir(self.deployment)
        already_set = os.path.join(copy_to, "www", "already_set")
        if not os.path.isfile(already_set):
            copy_dir = os.path.join(roslib.packages.get_pkg_dir("bellbot_gui"), "www")
            subprocess.call(["cp", "-r", copy_dir, copy_to])
            with open(already_set, "w") as f:
                f.write("")



    def manage(self, state):
        # print "STATE:", state.name
        try:
            self.states_cbs[state.name](state)
        except KeyError:
            rospy.logerr("Bellbot_GUI/manager: sad panda")
            strands_webserver.client_utils.display_url(display_no, random_page())

    def _on_node_shutdown(self):
        subprocess.call(["pkill", "florence"])
        strands_webserver.client_utils.display_url(display_no, random_page())


class GUI_Destination_Selection(object):
    def __init__(self, http_root, write_dir, deployment):
        self.http_root = http_root
        self.write_dir = write_dir
        self.deployment = deployment
        # self.categories = rospy.get_param('~destination_types')
        self.categories = rospy.get_param('/bellbot_gui/destinations_types')
        self.dests = self.get_metadata()
        ## self.dests_ab_sorted = sorted(self.dests.keys())
        self.dests_per_categ = self.sort_dests_per_categ()
        self.select_dest_page, self.page = self.make_page()
        self.sub = None

    def display(self, state=None):
        if self.sub is not None:
            self.sub.unregister()
        florence()
        strands_webserver.client_utils.display_relative_page(display_no, self.select_dest_page)
        # strands_webserver.client_utils.display_content(display_no, self.page) # hmmm does not display it relative to http_root

        self.sub = rospy.Subscriber("/bellbot_gui/sel_dest", String, self.sel_dest_cbk)

    def sel_dest_cbk(self, data):
        os.system("pkill florence")
        dest = data.data
        rospy.loginfo("Going to %s", dest)
        try:
          proxy = rospy.ServiceProxy('/bellbot_new_target', NewTarget)
          res = proxy(NewTargetRequest(dest))
          return res
          #return NewTargetResponse()
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e


    def sort_dests_per_categ(self, dests=None):
        if dests is None:
            dests = self.dests
        dests_per_categ = {}
        for c in self.categories:
            dests_per_categ[c] = {}
        for k, v in dests.items():
            dests_per_categ[v.kind][v.id] = v
            # print k, "//", v.id, "//", v.name, "//", v.kind
        # with open("dests_per_categ.p", "wb") as f:
        #     pickle.dump(dests_per_categ, f)
        return dests_per_categ

    def get_metadata(self):
        dests = {}
        rospy.wait_for_service('/query_node_metadata')
        try:
            proxy = rospy.ServiceProxy('/query_node_metadata', NodeMetadata)
            for c in self.categories:
                # print "Getting", c
                map_name = rospy.get_param("/topological_map_name","aaf_predep")
                res = proxy(NodeMetadataRequest("aaf_winter", map_name, c)) # 'office' | 'Meeting Rooms'
                for i in range(0, len(res.name)):
                    foo = res.name[i]
                    # foo = res.name[i].replace("-", "_")
                    dests[foo] = Destination_Data(name=foo, description=res.description[i], kind=res.node_type[i], goto=res.goto_node[i], available=True, at_node=res.at_node[i])

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return dests

    def make_page(self):
        cfg = ConfigParser.SafeConfigParser()
        if len(cfg.read(os.path.join(roslib.packages.get_pkg_dir(self.deployment), "conf", "locale.ini"))) == 0:
            raise IOError("locale.ini not found")

        gui_select_dest_template = os.path.join(roslib.packages.get_pkg_dir("bellbot_gui"), "templates", "gui_select_dest_template.html")

        select_dest_page_filename = "gui_select_dest_page.html"
        gui_select_dest_page = os.path.join(self.http_root, select_dest_page_filename)
        # gui_select_dest_page = os.path.join(self.write_dir, select_dest_page_filename)

        with open(gui_select_dest_template, "r") as f:
            data_gui_select_dest_template = f.read()
        # foo = str("file://" + os.path.join(roslib.packages.get_pkg_dir(self.deployment), "www", "images", cfg.get("select_destination_page", "logo")))
        foo = cfg.get("select_destination_page", "logo")
        data_gui_select_dest_template = data_gui_select_dest_template.replace("REPLACE_LOGO", foo)
        # data_gui_select_dest_template = data_gui_select_dest_template.replace("REPLACE_LOGO_ALT", cfg.get("select_destination_page", "logo_alt"))
        data_gui_select_dest_template = data_gui_select_dest_template.replace("REPLACE_WELCOME", cfg.get("select_destination_page", "welcome"))
        data_gui_select_dest_template = data_gui_select_dest_template.replace("REPLACE_BUTTON", cfg.get("select_destination_page", "button"))
        data_gui_select_dest_template = data_gui_select_dest_template.replace("REPLACE_NO_DESTINATION_SELECTED", cfg.get("select_destination_page", "no_destination_selected"), 1)

        options_str = ""
        for categ, rooms in self.dests_per_categ.items():
            options_str += '<optgroup label="' + categ + '">\n'
            rooms_ids = sorted(rooms.keys())
            for room_id in rooms_ids:
                options_str += self.ret_options_str(rooms[room_id])
            options_str += "</optgroup>\n"
        data_gui_select_dest_template = data_gui_select_dest_template.replace("REPLACE_DESTINATION_OPTIONS", options_str, 1)
        with open(gui_select_dest_page, "w") as f:
            f.write(data_gui_select_dest_template)
            # f.write(options_str)
            # f.write(data_gui_select_dest_footer)
        return select_dest_page_filename, data_gui_select_dest_template

    def ret_options_str(self, room):
        # print(room.name, room.goto)
        return '<option' + ' id="' + room.id + '" name="' + room.name + '" description="' + room.description + '" kind="' + room.kind + '" goto="' + room.goto + '" available="' + str(room.available) + '" at_node="' + str(room.at_node) +'">' + room.name + "</option>\n"
        # return '<option' + ' value="' + room.goto + '" id="' + room.id + '" name="' + room.name + '" description="' + room.description + '" kind="' + room.kind + '" goto="' + room.goto + '" available="' + str(room.available) + '" at_node="' + str(room.at_node) +'">' + room.name + "</option>\n"


# class GUI_Wait_For_Single_Guest(object):
#     def __init__(self):
#         self.service_prefix = '/bellbot_gui_services'
#         self.buttons = [('Go', 'trigger_confirm_single_guest')]
#         rospy.Service(self.service_prefix+'/trigger_confirm_single_guest', std_srvs.srv.Empty, self.trigger_go)
#
#     def display(self, state=None):
#         buttons_content = strands_webserver.page_utils.generate_alert_button_page("", self.buttons, self.service_prefix)
#         message = "<div>Click to confirm that you would like to go to " + state.goal + "</div>" + "<div>" + state.text + "</div>"
#         www_content = message + buttons_content
#         strands_webserver.client_utils.display_content(display_no, www_content)
#
#     def trigger_go(self, req):
#         rospy.wait_for_service('/bellbot_accept_target_single')
#         try:
#             proxy = rospy.ServiceProxy('/bellbot_accept_target_single', NewTarget)
#             res = proxy(NewTargetRequest(""))
#             return res
#         except rospy.ServiceException, e:
#             print "Service call failed: %s"%e
#
#
# class GUI_Wait_For_Multi_Guests(object):
#     def __init__(self):
#         self.service_prefix = '/bellbot_gui_services'
#         self.buttons = [('Go', 'trigger_confirm_multi_guests')]
#         rospy.Service(self.service_prefix+'/trigger_confirm_multi_guests', std_srvs.srv.Empty, self.trigger_go)
#
#     def display(self, state=None):
#         buttons_content = strands_webserver.page_utils.generate_alert_button_page("", self.buttons, self.service_prefix)
#         message = "<div>Click to confirm that you would like to go to " + state.goal + "</div>" + "<div>" + state.text + "</div>"
#         www_content = message + buttons_content
#         strands_webserver.client_utils.display_content(display_no, www_content)
#
#     def trigger_go(self, req):
#         rospy.wait_for_service('/bellbot_accept_target_mult')
#         try:
#             proxy = rospy.ServiceProxy('/bellbot_accept_target_mult', NewTarget)
#             res = proxy(NewTargetRequest(""))
#             return res
#         except rospy.ServiceException, e:
#             print "Service call failed: %s"%e


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
    def __init__(self, http_root, write_dir, deployment):
        self.sub = None
        self.http_root = http_root
        self.write_dir = write_dir
        self.deployment = deployment
        self.make_page()
        self.feedback_filename = os.path.join(self.write_dir, "user_feedback.csv")
        # rospy.Service('/bellbot/gui/feedback_done', std_srvs.srv.Empty, self.handle_feedback_done)

    def display(self, state=None):
        if self.sub is not None:
            self.sub.unregister()
        strands_webserver.client_utils.display_relative_page(display_no, 'feedback.html')
        self.sub = rospy.Subscriber("/bellbot_gui_feedback", String, self.cb_got_feedback)

    def make_page(self):
        cfg = ConfigParser.SafeConfigParser()
        if len(cfg.read(os.path.join(roslib.packages.get_pkg_dir(self.deployment), "conf", "locale.ini"))) == 0:
            raise IOError("locale.ini not found")

        template_file = os.path.join(roslib.packages.get_pkg_dir("bellbot_gui"), "templates", "feedback_template.html")
        target_path_file = os.path.join(self.http_root, "feedback.html")

        with open(template_file, "r") as f:
            template = f.read()
        template = template.replace("REPLACE_Q1", cfg.get("user_feedback_page", "q1"))
        template = template.replace("REPLACE_Q2", cfg.get("user_feedback_page", "q2"))
        template = template.replace("REPLACE_Q3", cfg.get("user_feedback_page", "q3"))
        template = template.replace("REPLACE_Q4", cfg.get("user_feedback_page", "q4"))
        template = template.replace("REPLACE_Q5", cfg.get("user_feedback_page", "q5"))

        with open(target_path_file, "w") as f:
            f.write(template)

    def cb_got_feedback(self, data):
        feedback = data.data
        feedback = feedback.split(",")
        for i in range(len(feedback)):
            feedback[i] = int(feedback[i])
        time_now_str = datetime.strftime(datetime.now(), '%Y-%m-%d %H:%M:%S')
        line_lst = [time_now_str] + feedback
        with open(self.feedback_filename, "a") as f:
            writer = csv.writer(f, delimiter=",", quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
            writer.writerow(line_lst)
        self.handle_feedback_done()

    def handle_feedback_done(self):
        rospy.wait_for_service('/bellbot_feedback')
        try:
            proxy = rospy.ServiceProxy('/bellbot_feedback', std_srvs.srv.Empty)
            proxy(std_srvs.srv.EmptyRequest())
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return



if __name__ == '__main__':
    rospy.init_node("bellbot_gui")
    # The display to publish on, defaulting to all displays
    display_no = rospy.get_param("~display", 0)

    gui = Bellbot_GUI()
    rospy.spin()
