#!/usr/bin/python

import rospy
import roslib

import web
import signal
from os import chdir
from os.path import join

### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('bellbot_gui') + '/www'
WEBTOOLS_DIR = roslib.packages.get_pkg_dir('strands_webtools')

html_config = {
    'rosws_suffix': ':9090'
}

destination_waypoint = ''

available_destinations = ['test', 'tst2']

render = web.template.render(TEMPLATE_DIR, base='base', globals=globals())

chdir(TEMPLATE_DIR)


class ControlServer(web.application):
    def __init__(self):
        urls = (
            '/WaitingForGoal', 'DestinationPage',
            '/SetGoal', 'SetGoalPage',
            '/Guiding', 'NavigationPage',
            '/WaitingForFeedback', 'FeedbackPage',
            '/webtools/(.*)', 'Webtools'
        )

        print globals()
        web.application.__init__(self, urls, globals())
        signal.signal(signal.SIGINT, self.signal_handler)

    def run(self, port=8027, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', port))

    def signal_handler(self, signum, frame):
        self.stop()


class DestinationPage(object):
    def GET(self):
        return render.destination()


class NavigationPage(object):
    def GET(self):
        user_data = web.input()
        rospy.loginfo("requested to go to %s" % user_data.destination)
        html_config['destination_waypoint'] = user_data.destination
        return render.navigation()

class SetGoalPage(object):
    def GET(self):
        user_data = web.input()
        rospy.loginfo("requested to go to %s" % user_data.destination)
        html_config['destination_waypoint'] = user_data.destination
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
            web.application.notfound(app)  # file not found


if __name__ == "__main__":
    rospy.init_node("bellbot_gui")
    port = rospy.get_param('~port', 8223)

    rospy.loginfo("bellbot_gui started.")
    app = ControlServer()
    app.run(port=port)
