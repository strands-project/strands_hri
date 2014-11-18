#! /usr/bin/env python

import rospy
import smach
import yaml
import actionlib
from math import hypot, atan
from geometry_msgs.msg import Pose, Point32, Point
from bayes_people_tracker.msg import PeopleTracker
from strands_human_following.wander_search import point_inside_poly
from strands_navigation_msgs.msg import MonitoredNavigationAction
from strands_navigation_msgs.msg import MonitoredNavigationGoal
from scitos_ptu.msg import PtuGotoAction, PtuGotoGoal
from tf.transformations import euler_from_quaternion


class Follow(smach.State):

    def __init__(self):
        rospy.loginfo('Entering follow state...')
        self._load_params()
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['current_robot', 'current_pose_tf', 'id_now'],
            output_keys=['degree_to_go'])

        self.old_angle = 0

        self.monav_client = actionlib.SimpleActionClient(
            'monitored_navigation', MonitoredNavigationAction)
        rospy.loginfo('Waiting for monitored navigation server...')
        self.monav_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo('Connected to monitored navigation server...')

        self.pan_client = actionlib.SimpleActionClient(
            'SetPTUState', PtuGotoAction)
        rospy.loginfo('Waiting for pan tilt server...')
        self.pan_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo('Connected to pan tilt server...')

    def _load_params(self):
        config = yaml.load(open(rospy.get_param("~config_file")))
        self.alpha = config['alpha']
        self.default_distance = config['distance']

        # Getting polygon from launch file
        self.points = []

        for point in config['follow_area']:
            rospy.loginfo('Points: %s', point)
            self.points.append(Point32(float(point[0]), float(point[1]), 0))

    def execute(self, userdata):
        while not hasattr(userdata, 'id_now'):
            rospy.sleep(rospy.Duration(0.1))

        if userdata.id_now == -1:
            self.old_angle = 0
            rospy.sleep(rospy.Duration(0.3))

        move_goal = MonitoredNavigationGoal()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.header.stamp = rospy.Time.now()
        move_goal.action_server = 'move_base'

        delta_x = userdata.current_pose_tf.position.x - \
            userdata.current_robot.position.x
        delta_y = userdata.current_pose_tf.position.y - \
            userdata.current_robot.position.y
        distance = hypot(delta_x, delta_y)
        quaternion = [
            userdata.current_robot.orientation.x,
            userdata.current_robot.orientation.y,
            userdata.current_robot.orientation.z,
            userdata.current_robot.orientation.w]
        robot_angle = euler_from_quaternion(quaternion, 'rxyz')
        human_angle = atan(delta_y / delta_x)

        if delta_x < 0:
            if delta_y > 0:
                human_angle = human_angle + 3.14
            else:
                human_angle = human_angle - 3.14

        delta_angle = human_angle - robot_angle[2]
        if delta_angle < -3.14:
            delta_angle = delta_angle + 6.28
        if delta_angle > 3.14:
            delta_angle = delta_angle - 6.28
        delta_angle_degree = delta_angle / 6.28 * 360

        p = Point(userdata.current_pose_tf.position.x,
                  userdata.current_pose_tf.position.y,
                  userdata.current_pose_tf.position.z)

        if distance <= self.default_distance or \
                not point_inside_poly(p, self.points):
            move_goal.target_pose.pose = userdata.current_robot
        else:
            move_goal.target_pose.pose = userdata.current_pose_tf

        self.monav_client.send_goal(move_goal)
        self.monav_client.wait_for_result(rospy.Duration(1))

        if delta_angle_degree >= 20 or delta_angle_degree <= -20:
            pan_goal = PtuGotoGoal()
            temp = self.old_angle * self.alpha
            temp1 = delta_angle_degree * (1 - self.alpha)
            pan_goal.pan = temp + temp1
            pan_goal.tilt = 0
            pan_goal.pan_vel = 50
            pan_goal.tilt_vel = 10

            self.pan_client.send_goal(pan_goal)
            self.pan_client.wait_for_result(rospy.Duration(1))
            self.old_angle = pan_goal.pan

        userdata.degree_to_go = delta_angle_degree
        rospy.loginfo("Robot is in new position...")
        return 'succeeded'


class MoveSearch(smach.State):

    def __init__(self):
        rospy.loginfo('Entering follow state...')
        self._load_params()
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            output_keys=['current_robot', 'current_pose_tf', 'id_now'])

        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
        rospy.Subscriber('/people_tracker/positions', PeopleTracker,
                         self.people_pose_cb)

        self.current_robot = Pose()
        self.current_pose = Pose()
        self.current_pose_tf = Pose()
        self.last_move_pose = Pose()
        self.last_post = Pose()
        self.last_move_time = rospy.get_time()
        self.is_received = bool()
        self.suspend = 0
        self.id_now = -1

    def _load_params(self):
        config = yaml.load(open(rospy.get_param("~config_file")))
        self.max_t_frames = config['max_t_frames']

    def execute(self, userdata):
        status = 'aborted'
        count = 0
        while True:
            count = count + 1
            if self.is_received:
                userdata.current_robot = self.current_robot
                userdata.current_pose_tf = self.current_pose_tf
                userdata.id_now = self.id_now
                time = rospy.get_time() - self.last_move_time
                if time > 20:
                    break
                status = 'succeeded'

            if count >= 10000:
                break

        return status

    def robot_pose_cb(self, data):
        self.current_robot = data

    def people_pose_cb(self, data):
        if self.id_now == -1:
            if len(data.uuids) == 0:
                self.is_received = False
            else:
                self.id_now = data.uuids[
                    data.distances.index(min(data.distances))]
        elif self.id_now in data.uuids:
            self.suspend = 0
            self.last_post = self.current_pose_tf
            self.current_pose_tf = data.poses[data.uuids.index(self.id_now)]

            if hypot(self.last_move_pose.position.x -
                     self.current_pose_tf.position.x,
                     self.last_move_pose.position.y -
                     self.current_pose_tf.position.y) > 0.23:
                self.last_move_pose = self.current_pose_tf
                self.last_move_time = rospy.get_time()

            self.is_received = True
        else:
            self.suspend += 1

            if self.suspend >= self.max_t_frames:
                self.is_received = False
                self.id_now = -1
                self.suspend = 0
            else:
                self.is_received = True
