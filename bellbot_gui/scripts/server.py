#!/usr/bin/python

import rospy
import roslib

import web
import signal

import strands_webserver.page_utils
import strands_webserver.client_utils

import threading
import json

from mongodb_store.message_store import MessageStoreProxy

from os import chdir
from os.path import join

from bellbot_action_server.srv import NewTarget, NewTargetRequest
from bellbot_action_server.msg import BellbotState
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import String
from topological_utils.srv import NodeMetadata, NodeMetadataRequest, NodeMetadataResponse


### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('bellbot_gui') + '/www'
WEBTOOLS_DIR = roslib.packages.get_pkg_dir('strands_webtools')

html_config = {
    'rosws_suffix': ':9090',
    'destination_waypoint': 'not defined',
    'questions': [
        'Ich bin mit dem Botenservice des Roboters zufrieden',
        'Es ist praktisch einen Botenroboter zu haben',
        'Ich wuerde den Botenroboter wieder benutzen',
        'Ich fand den Botenroboter einfach zu bedienen',
        'Ich war mit der Geschwindigkeit des Roboters zufrieden'
    ]
}

destination_waypoint = ''

available_destinations = ['test', 'tst2']

render = web.template.render(TEMPLATE_DIR, base='base', globals=globals())

chdir(TEMPLATE_DIR)


class ControlServer(web.application):
    def __init__(self):
        self.gui_host = rospy.get_param('~gui_host', 'localhost')
        self.gui_port = rospy.get_param('~gui_port', '8223')
        self.gui_prefix = "http://%s:%s/" % (self.gui_host, self.gui_port)

        urls = (
            '/Setup', 'SetupPage',
            '/WaitingForGoal', 'DestinationPage',
            '/SetGoal', 'SetGoalPage',
            '/Guiding', 'NavigationPage',
            '/WaitingForFeedback', 'FeedbackPage',
            '/webtools/(.*)', 'Webtools'
        )

        self._feedback_msg_store = MessageStoreProxy(collection='bellbot_feedback')
        self._dest_pub = rospy.Publisher('~destination', String, queue_size=1, latch=True)
        rospy.Subscriber("/bellbot_state", BellbotState, self.manage)
        rospy.Subscriber("/bellbot_gui_feedback", String, self.store_feedback)

        web.application.__init__(self, urls, globals())
        signal.signal(signal.SIGINT, self.signal_handler)

    def store_feedback(self, feedback):
        #rospy.wait_for_service('/bellbot_feedback')
        service = rospy.ServiceProxy('/bellbot_feedback', Empty)
        request = EmptyRequest()
        self._feedback_msg_store.insert(feedback)
        rospy.loginfo('feedback stored: %s' % feedback)

        try:
            service.call(request)
        except Exception, e:
            rospy.logerr("failed to call /bellbot_feedback: %s" % e)


    def manage(self, state):
        rospy.loginfo("STATE: %s" % state.name)
        if state.goal is not None:
            self._dest_pub.publish(state.goal)
        try:
            url = "%s%s?destination=%s" % (self.gui_prefix, state.name, state.goal)
            strands_webserver.client_utils.display_url(0, url)
        except Exception:
            rospy.logerr("Bellbot_GUI/manager: no page for %s" % state.name)

    def run(self, port=8027, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', port))

    def signal_handler(self, signum, frame):
        self.stop()


class DestinationPage(object):

    def __init__(self):
        self.categories = rospy.get_param('/bellbot_gui/destinations_types',
                                          ["department", "person", "office", "Meeting Rooms"])

    def get_metadata(self):
        dests = {}
        #rospy.wait_for_service('/query_node_metadata')
        try:
            proxy = rospy.ServiceProxy('/query_node_metadata', NodeMetadata)

            for c in self.categories:
                # print "Getting", c
                map_name = rospy.get_param("/topological_map_name","aaf_predep")
                res = proxy(NodeMetadataRequest(map_name, c)) # 'office' | 'Meeting Rooms'
                print res
                for i in range(0, len(res.name)):
                    foo = res.name[i].decode('utf-8')
                    dests[foo] = res.goto_node[i]

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return dests


    def GET(self):
        dests = self.get_metadata()
        html_config['dests'] = dests

        html_config['available_destinations'] = json.dumps(dests.keys()) #[simplejson.dumps(s) for s in dests.keys()]
        print "destionations: " + html_config['available_destinations']
        return render.destination()


class NavigationPage(object):
    def GET(self):
        return render.navigation()

class SetupPage(object):
    def GET(self):
        return render.setup()


class SetGoalPage(object):
    def GET(self):
        user_data = web.input()
        try:
            waypoint = html_config['dests'][user_data.destination]
            rospy.loginfo("requested to go to %s (%s)" % (user_data.destination, waypoint))

            html_config['destination_waypoint'] = user_data.destination
            rospy.wait_for_service('/bellbot_new_target')

            target_service = rospy.ServiceProxy('/bellbot_new_target', NewTarget)

            request = NewTargetRequest()
            request.target = waypoint
            rospy.loginfo("calling to go to waypoint %s" % waypoint)
            rospy.loginfo(target_service.call(request))
        except Exception, e:
            rospy.logerr('error when calling bellbot service. %s' % e)
            return web.seeother('/WaitingForGoal')
        return render.waiting()


class FeedbackPage(object):
    def GET(self):
        return render.feedback()


class Webtools(object):
    """
    proxies all requests to strands_webtools
    """
    def GET(self, f):
        try:
            p = join(WEBTOOLS_DIR, f)
            rospy.logdebug("trying to serve %s from %s", f, p)
            if f.endswith('.js'):
                web.header('Content-Type', 'text/javascript')
            return open(p, 'r').read()
        except:
            web.application.notfound(app)


if __name__ == "__main__":
    rospy.init_node("bellbot_gui")
    port = rospy.get_param('~port', 8223)
    app = ControlServer()
    app.run(port=port)

    rospy.loginfo("bellbot_gui started.")
