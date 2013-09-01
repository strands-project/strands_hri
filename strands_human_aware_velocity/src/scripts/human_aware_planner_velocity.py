#!/usr/bin/env python

import rospy
import actionlib
import dynamic_reconfigure.client
from strands_perception_people_msgs.msg import PedestrianLocations
import strands_human_aware_velocity.msg

class DynamicVelocityReconfigure():
    "A calss to reconfigure the velocity of the DWAPlannerROS."

    # Create feedback and result messages
    _feedback = strands_human_aware_velocity.msg.HumanAwareVelocityFeedback()
    _result   = strands_human_aware_velocity.msg.HumanAwareVelocityResult()

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, strands_human_aware_velocity.msg.HumanAwareVelocityAction, execute_cb = None, auto_start = False)
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        sub_topic = rospy.get_param("~pedestrian_locations", '/pedestrian_localisation/localisations')
        rospy.Subscriber(sub_topic, PedestrianLocations, self.pedestrianCallback, None, 5)
        self.fast = True
        rospy.loginfo("Creating dynamic reconfigure client")
        self.client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
        rospy.loginfo(" ...done")

    def resetSpeed(self):
        rospy.logdebug("Resetting speeds to max:")
        rospy.loginfo(" Setting parameters: %s", self.fast_param)
        self.client.update_configuration(self.fast_param)

    def pedestrianCallback(self, pl):
        if not self._as.is_active():
            rospy.logdebug("No active goal")
            return

        self._feedback.num_found_humans = len(pl.poses)
        self._feedback.min_dist = 0.0

        if len(pl.poses) > 0:
            rospy.logdebug("Found pedestrian: ")
            rospy.logdebug(" Pedestrian distance: %s", pl.min_distance)
            self._feedback.min_dist = pl.min_distance
            factor = pl.min_distance - self.min_dist
            factor = factor if factor > 0.0 else 0.0
            factor /= (self.max_dist - self.min_dist)
            factor = 1.0 if factor > 1.0 else factor
            trans_speed = factor * self.max_vel_x
            trans_speed = round(trans_speed, 2)
            rospy.logdebug("Calculated translational speed: %s", trans_speed)
            self._feedback.current_speed = trans_speed
            rot_speed = factor * self.max_rot_vel
            rot_speed = round(rot_speed, 2)
            rospy.logdebug("Calculated rotaional speed: %s", rot_speed)
            self._feedback.current_rot = rot_speed
            if not trans_speed == self.max_vel_x and not rot_speed == self.max_rot_vel:
                self.slow_param = {'max_vel_x' : trans_speed, 'max_trans_vel' : trans_speed, 'max_rot_vel' : rot_speed}
                self.client.update_configuration(self.slow_param)
                rospy.logdebug(" Setting parameters: %s", self.slow_param)
                self.fast = False
            self.timeout = rospy.get_time() + self.threshold
        elif rospy.get_time() > self.timeout:
            rospy.logdebug("Not found any pedestrians:")
            self._feedback.current_speed = self.max_vel_x
            if not self.fast:
                self.resetSpeed()
                self.fast = True
            else:
                rospy.logdebug(" Already fast")

        self._feedback.remaining_runtime = self.end_time - rospy.get_time() if self.end_time > 0 else float('Inf')
        self._feedback.time_to_reset = self.timeout - rospy.get_time()
        self._feedback.time_to_reset = self._feedback.time_to_reset if self._feedback.time_to_reset > 0.0 else 0.0
        self._as.publish_feedback(self._feedback)

        if rospy.get_time() > self.end_time and self.end_time > 0:
            rospy.loginfo("Execution time has been reached. Goal terminated successfully")
            self.resetSpeed()
            self._result.expired = True
            self._as.set_succeeded(self._result)

    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        rospy.loginfo("Received goal:\n%s", self._goal)
        self.threshold = self._goal.time_to_reset
        current_time = rospy.get_time()
        self.timeout = current_time + self.threshold
        self.end_time = current_time + self._goal.seconds if self._goal.seconds > 0 else -1.0
        self.max_vel_x = round(self._goal.max_vel_x, 2)
        self.max_rot_vel = round(self._goal.max_rot_vel, 2)
        self.max_dist = self._goal.max_dist
        self.min_dist = self._goal.min_dist
        self.fast_param = {'max_vel_x' : self.max_vel_x, 'max_trans_vel' : self.max_vel_x, 'max_rot_vel' : self.max_rot_vel}
        self._feedback.current_speed = self.max_vel_x
        self.resetSpeed()

    def preemptCallback(self):
        rospy.loginfo("Cancelled execution of goal:\n%s", self._goal)
        self.resetSpeed()
        self._result.expired = False
        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node("human_aware_planner_velocities")
    dvr = DynamicVelocityReconfigure(rospy.get_name())
    rospy.spin()
