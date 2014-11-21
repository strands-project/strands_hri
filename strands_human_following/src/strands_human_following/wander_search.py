#! /usr/bin/env python

import rospy
import yaml
import smach
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Polygon, Pose, Point32
from nav_goals_generator.srv import NavGoals
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
        self.is_received = bool()
        self.current_uuid = -1
        self.current_robot = Pose()
        self.current_pose_tf = Pose()
        self.mode = rospy.get_param('~wandering_mode', 'wait')

        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'])

        rospy.Subscriber('/people_tracker/positions',
                         PeopleTracker, self.people_pose_cb)
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
        rospy.loginfo('Waiting for navigation service...')
        rospy.wait_for_service('nav_goals')

        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        self.client = actionlib.SimpleActionClient(
            'monitored_navigation', MonitoredNavigationAction)
        self.client.wait_for_server(rospy.Duration(60))
        rospy.loginfo('Receiving navigation goals...')

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
        status = 'aborted'
        rospy.loginfo('Enter wandering mode... ')

        monav_goal = MonitoredNavigationGoal()
        monav_goal.target_pose.header.frame_id = 'map'
        monav_goal.target_pose.header.stamp = rospy.Time.now()
        monav_goal.action_server = 'move_base'

        if self.mode == 'normal':
            status, userdata = self.wander_mode(monav_goal, userdata)
        elif point_inside_poly(self.wait_point.position, self.points):
            status, userdata = self.wait_mode(monav_goal, userdata)

        if self.preempt_requested():
            self.service_preempt()
            status = 'preempted'

        self.current_uuid = -1
        return status

    def request_preempt(self):
        smach.State.request_preempt(self)
        rospy.logwarn("Wandering mode is preempted!")

    def wander_mode(self, monav_goal, userdata):
        status = 'aborted'
        while not self.preempt_requested() and status != 'succeeded':
            nav_goals = self.nav_goals(1, 0.7, self.polygon)
            monav_goal.target_pose.pose = nav_goals.goals.poses[0]
            self.client.send_goal(monav_goal)
            self.client.wait_for_result(rospy.Duration(1))
            while not self.preempt_requested() and \
                    self.client.get_state() == GoalStatus.ACTIVE:
                if self.is_received:
                    self.client.cancel_goal()
                    status = 'succeeded'
                    # userdata.current_uuid = self.current_uuid
                    # userdata.current_robot = self.current_robot
                    # userdata.current_pose_tf = self.current_pose_tf
                    break

        return status, userdata

    def wait_mode(self, monav_goal, userdata):
        status = 'aborted'
        monav_goal.target_pose.pose = self.wait_point
        self.client.send_goal(monav_goal)
        self.client.wait_for_result(rospy.Duration(1))
        while not self.preempt_requested():
            if self.is_received:
                if self.client.get_state() == GoalStatus.ACTIVE:
                    self.client.cancel_goal()
                status = 'succeeded'
                # userdata.current_uuid = self.current_uuid
                # userdata.current_robot = self.current_robot
                # userdata.current_pose_tf = self.current_pose_tf
                break

        return status, userdata

    def people_pose_cb(self, data):
        if len(data.uuids) == 0:
            self.is_received = False
        else:
            self.current_uuid = \
                data.uuids[data.distances.index(min(data.distances))]
            self.current_pose_tf = \
                data.poses[data.uuids.index(self.current_uuid)]
            self.is_received = True

    def robot_pose_cb(self, data):
        self.current_robot = data
