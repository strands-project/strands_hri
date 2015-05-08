#!/usr/bin/python

import rospy
import roslib

import web
import signal
from os import chdir
from os.path import join
from aaf_control_ui.srv import DemandTask
from aaf_control_ui.srv import DemandTaskResponse
from strands_executive_msgs.srv import DemandTask as SchedulerDemandTask
from strands_executive_msgs.srv import DemandTaskRequest as SchedulerDemandTaskRequest

### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('aaf_control_ui') + '/www'
WEBTOOLS_DIR = roslib.packages.get_pkg_dir('strands_webtools')

render = web.template.render(TEMPLATE_DIR, base='base')
chdir(TEMPLATE_DIR)


class ControlServer(web.application):
    def __init__(self):
        urls = (
            '/', 'DashboardPage',
            '/tasks', 'TasksPage',
            '/setup', 'SetupPage',
            '/help', 'HelpPage',
            '/webtools/(.*)', 'Webtools'
        )
        web.application.__init__(self, urls, globals())
        rospy.Service(rospy.get_name()+'/demand_task', DemandTask, self.demand_task)
        signal.signal(signal.SIGINT, self.signal_handler)

    def run(self, port=8027, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', port))

    def signal_handler(self, signum, frame):
        self.stop()
        print "aaf_control_server stopped."

    def demand_task(self, req):
        r = SchedulerDemandTaskRequest()
        r.task.action = req.action
        r.task.start_node_id = req.waypoint
        r.task.end_node_id = req.waypoint
        rospy.loginfo('demanding task %s now' % r.task.action)
        dt = rospy.ServiceProxy('/task_executor/demand_task', SchedulerDemandTask)
        dt.wait_for_service()
        dt.call(r)
        return DemandTaskResponse()
        
        

class DashboardPage(object):
    def GET(self):
        return render.dashboard()


class TasksPage(object):
    def GET(self):
        return render.tasks()


class SetupPage(object):
    def GET(self):
        return render.setup()


class HelpPage(object):
    def GET(self):
        return render.help()


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
    rospy.init_node("aaf_control_server")
    port = rospy.get_param('~port', 8127)

    rospy.loginfo("aaf_control_server started.")
    app = ControlServer()
    app.run(port=port)
