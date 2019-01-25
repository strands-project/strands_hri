#!/usr/bin/env python

import rospy
import actionlib
from dynamic_reconfigure.server import Server as DynServer
from dynamic_reconfigure.client import Client as DynClient
from bayes_people_tracker.msg import PeopleTracker
from strands_human_aware_navigation.cfg import HumanAwareNavigationConfig
import move_base_msgs.msg
import numpy as np
import actionlib_msgs.msg
import strands_gazing.msg


class DynamicVelocityReconfigure():
    "A class to reconfigure the velocity of the DWAPlannerROS."

    GAZE, NO_GAZE = range(2)

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        self.fast = True
        self.gaze_type = DynamicVelocityReconfigure.GAZE
        rospy.loginfo("Creating dynamic reconfigure client")
        self.client = DynClient(
            "/move_base/DWAPlannerROS"
        )
        rospy.loginfo(" ...done")
        rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient(
            'move_base',
            move_base_msgs.msg.MoveBaseAction
        )
        self.baseClient.wait_for_server()
        rospy.loginfo("Creating gaze client.")
        self.gazeClient = actionlib.SimpleActionClient(
            'gaze_at_pose',
            strands_gazing.msg.GazeAtPoseAction
        )
        self.gazeClient.wait_for_server()
        rospy.loginfo("...done")

        # Magic numbers overwritten in dyn_callback
        self.threshold = 4.0
        self.max_dist = 5.0
        self.min_dist = 1.5
        self.detection_angle = 90.0

        # More Magic numbers by @jailander
        self.fast_param = {
            "max_vel_x" : 0.55,
            "max_trans_vel" : 0.55,
            "max_rot_vel" : 1.0
        }


        self.dyn_srv = DynServer(HumanAwareNavigationConfig, self.dyn_callback)

        current_time = rospy.get_time()
        self.timeout = current_time + self.threshold
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            move_base_msgs.msg.MoveBaseAction,
            self.goalCallback,
            auto_start=False
        )

        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        self.sub_topic = rospy.get_param(
            "~people_positions",
            '/people_tracker/positions'
        )
        self.ppl_sub = None

        self.last_cancel_time = rospy.Time(0)
        rospy.Subscriber(
            "/human_aware_navigation/cancel",
            actionlib_msgs.msg.GoalID,
            self.cancel_time_checker_cb
        )

    def dyn_callback(self, config, level):
        if config["gaze_type"] == DynamicVelocityReconfigure.NO_GAZE:
            self.cancel_gaze_goal()
        self.gaze_type       = config["gaze_type"]
        self.threshold       = config["timeout"]
        self.max_dist        = config["max_dist"]
        self.min_dist        = config["min_dist"]
        self.detection_angle = config["detection_angle"]
        max_vel_x            = config["max_vel_x"]
        max_trans_vel        = config["max_trans_vel"]
        max_rot_vel          = config["max_rot_vel"]
        self.fast_param = {
            "max_vel_x" : max_vel_x,
            "max_trans_vel" : max_trans_vel,
            "max_rot_vel" : max_rot_vel
        }
        return config

    def cancel_time_checker_cb(self, msg):
        self.last_cancel_time = rospy.get_rostime()

    def send_gaze_goal(self, topic):
        if self.gaze_type == DynamicVelocityReconfigure.GAZE:
            gaze_goal = strands_gazing.msg.GazeAtPoseGoal()
            gaze_goal.runtime_sec = 0
            gaze_goal.topic_name = topic
            self.gazeClient.send_goal(gaze_goal)

    def cancel_gaze_goal(self):
        if self.gaze_type == DynamicVelocityReconfigure.GAZE:
            self.gazeClient.cancel_all_goals()

    def resetSpeed(self):
        rospy.logdebug("Resetting speeds to max:")
        rospy.logdebug(" Setting parameters: %s", self.fast_param)
        try:
            self.client.update_configuration(self.fast_param)
        except rospy.ServiceException as exc:
            rospy.logerr("Caught service exception: %s", exc)
            try:
                self.client.update_configuration(self.fast_param)
            except rospy.ServiceException as exc:
                rospy.logerr("Caught service exception: %s", exc)
                try:
                    self.client.update_configuration(self.fast_param)
                except rospy.ServiceException as exc:
                    rospy.logerr("Caught service exception: %s", exc)
                    self.baseClient.cancel_all_goals()
                    self.cancel_gaze_goal()
                    self._as.set_aborted()

    def get_min_dist(self, data, angle):
        rad = angle * (np.pi / 180.0)
        distances = [x for idx, x in enumerate(data.distances) if data.angles[idx] >= -rad and data.angles[idx] <= rad]
        return np.min(distances) if len(distances) else 1000.0

    def pedestrianCallback(self, pl):
        if not self._as.is_active():
            rospy.logdebug("No active goal. Unsubscribing.")
            if self.ppl_sub:
                self.ppl_sub.unregister()
                self.ppl_sub = None
            return

        if len(pl.poses) > 0:
            rospy.logdebug("Found people: ")
            min_distance = self.get_min_dist(pl, self.detection_angle)
            rospy.logdebug(" People distance: %s", min_distance)
            factor = min_distance - self.min_dist
            factor = factor if factor > 0.0 else 0.0
            factor /= (self.max_dist - self.min_dist)
            factor = 1.0 if factor > 1.0 else factor
            trans_speed = factor * self.fast_param['max_vel_x']
            trans_speed = round(trans_speed, 2)
            rospy.logdebug("Calculated translational speed: %s", trans_speed)
            rot_speed = factor * self.fast_param['max_rot_vel']
            rot_speed = round(rot_speed, 2)
            rospy.logdebug("Calculated rotaional speed: %s", rot_speed)
            if not trans_speed == self.fast_param['max_vel_x']:  # and not rot_speed == self.fast_param['max_rot_vel']:
                self.send_gaze_goal("/upper_body_detector/closest_bounding_box_centre")
                self.slow_param = {
                    'max_vel_x': trans_speed,
                    'max_trans_vel': trans_speed,
                    'max_rot_vel': 0.0 if trans_speed < 0.05 else self.fast_param["max_rot_vel"]
                }
                try:
                    print 'making it slow'
                    self.client.update_configuration(self.slow_param)
                except rospy.ServiceException as exc:
                    rospy.logerr("Caught service exception: %s", exc)
                rospy.logdebug(" Setting parameters: %s", self.slow_param)
                self.fast = False
            self.timeout = rospy.get_time() + self.threshold
        elif rospy.get_time() > self.timeout:
            rospy.logdebug("Not found any pedestrians:")
            if not self.fast:
                self.send_gaze_goal("/pose_extractor/pose")
                self.resetSpeed()
                self.fast = True
            else:
                rospy.logdebug(" Already fast")

    def goalCallback(self, goal):
        self.ppl_sub = rospy.Subscriber(
            self.sub_topic,
            PeopleTracker,
            self.pedestrianCallback,
            None,
            1
        )
        self._goal = goal
        self.send_gaze_goal("/pose_extractor/pose")
        rospy.logdebug("Received goal:\n%s", self._goal)
        self.resetSpeed()

        self.moveBaseThread(self._goal)

    def preemptCallback(self):
        if rospy.get_rostime()-self.last_cancel_time < rospy.Duration(1):
            rospy.logdebug("Cancelled execution of goal:\n%s", self._goal)
            self.baseClient.cancel_all_goals()
            self.cancel_gaze_goal()
            self.resetSpeed()
        self._as.set_preempted()

    def moveBaseThread(self, goal):
        ret = self.moveBase(goal)
        self.resetSpeed()
        self.cancel_gaze_goal()
        if not self._as.is_preempt_requested() and ret:
            self._as.set_succeeded(self.result)
        elif not self._as.is_preempt_requested() and not ret:
            self._as.set_aborted(self.result)

    def moveBase(self, goal):
        rospy.logdebug('Moving robot to goal: %s', goal)
        self.baseClient.send_goal(goal, feedback_cb=self.moveBaseFeedbackCallback)
        status = self.baseClient.get_state()
        while status == actionlib_msgs.msg.GoalStatus.PENDING or status == actionlib_msgs.msg.GoalStatus.ACTIVE:
            status = self.baseClient.get_state()
            self.baseClient.wait_for_result(rospy.Duration(0.2))
            self.result = self.baseClient.get_result()
            if self._as.is_preempt_requested():
                self.preemptCallback()
                break

        if self.baseClient.get_state() != actionlib_msgs.msg.GoalStatus.SUCCEEDED and self.baseClient.get_state() != actionlib_msgs.msg.GoalStatus.PREEMPTED:
            return False

        #avoid jumping out of a state immediately after entering it - actionlib bug
        rospy.sleep(rospy.Duration.from_sec(0.3))
        return True

    def moveBaseFeedbackCallback(self, fb):
        self._as.publish_feedback(fb)

if __name__ == '__main__':
    rospy.init_node("human_aware_navigation")
    dvr = DynamicVelocityReconfigure(rospy.get_name())
    rospy.spin()
