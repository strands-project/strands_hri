#! /usr/bin/env python

import rospy
import yaml
import smach
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Polygon, Pose, Point32
from nav_goals_msgs.srv import NavGoals
from strands_navigation_msgs.msg import MonitoredNavigationAction
from strands_navigation_msgs.msg import MonitoredNavigationGoal
from bayes_people_tracker.msg import PeopleTracker


def point_inside_poly(point, poly):
    inside = False
    point1 = poly[0]

    for i in range(len(poly)+1):
        point2 = poly[i % len(poly)]
        if point.y > min(point1.y, point2.y):
            if point.y <= max(point1.y, point2.y):
                if point.x <= max(point1.x, point2.x):
                    if point1.y != point2.y:
                        xinters = (point.y - point1.y) * (point2.x - point1.x) \
                            / (point2.y - point1.y) + point1.x
                    if point1.x == point2.x or point.x <= xinters:
                        inside = not inside
        point1 = point2

    return inside


class Wander(smach.State):

    def __init__(self):
        rospy.loginfo('Entering wander state...')
        self._load_params()
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'])
        rospy.loginfo('Waiting for navigation service...')
        rospy.wait_for_service('nav_goals')

        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        rospy.loginfo('Receiving navigation goals...')
        self.mode = rospy.get_param('~wandering_mode', 'wait')

    def _load_params(self):
        config = yaml.load(open(rospy.get_param("~config_file")))

        # Getting polygon from launch file
        self.points = []

        for point in config['wander_area']:
            rospy.loginfo('Points: %s', point)
            self.points.append(Point32(float(point[0]), float(point[1]), 0))

        self.polygon = Polygon(self.points)

        # Getting wait point
        self.wait_point = Pose()
        wp_array = config['wait_point']
        self.wait_point.position.x = wp_array[0]
        self.wait_point.position.y = wp_array[1]
        self.wait_point.position.z = wp_array[2]
        self.wait_point.orientation.x = wp_array[3]
        self.wait_point.orientation.y = wp_array[4]
        self.wait_point.orientation.z = wp_array[5]
        self.wait_point.orientation.w = wp_array[6]

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            rospy.loginfo('Reseting preempt...')

        rospy.loginfo('Enter wandering mode... ' +
                      str(self.preempt_requested()))

        while not self.preempt_requested():
            nav_goals = self.nav_goals(1, 0.7, self.polygon)
            client = actionlib.SimpleActionClient(
                'monitored_navigation', MonitoredNavigationAction)
            client.wait_for_server(rospy.Duration(60))

            monav_goal = MonitoredNavigationGoal()
            monav_goal.target_pose.header.frame_id = 'map'
            monav_goal.target_pose.header.stamp = rospy.Time.now()
            monav_goal.action_server = 'move_base'

            if self.mode == 'normal':
                monav_goal.target_pose.pose = nav_goals.goals.poses[0]
                client.send_goal(monav_goal)
            elif point_inside_poly(self.wait_point.position, self.points):
                monav_goal.target_pose.pose = self.wait_point
                client.send_goal(monav_goal)

            client.wait_for_result(rospy.Duration(10))
            while not self.preempt_requested() and \
                    client.get_state() == GoalStatus.ACTIVE:
                rospy.sleep(rospy.Duration(0.5))
            client.cancel_goal()

            rospy.loginfo('Waiting for move base...')

        return 'succeeded'


class Search(smach.State):

    def __init__(self):
        rospy.loginfo('Entering search state...')
        smach.State.__init__(
            self, outcomes=['succeeded', 'aborted', 'preempted'])

        rospy.Subscriber('/people_tracker/positions',
                         PeopleTracker, self.people_pose_cb)

        self.is_received = bool()

    def execute(self, userdata):
        status = 'aborted'
        while(True):
            if self.is_received:
                status = 'succeeded'
                break
            rospy.sleep(rospy.Duration(0.5))
        return status

    def people_pose_cb(self, data):
        if len(data.uuids) == 0:
            self.is_received = False
        else:
            self.is_received = True
