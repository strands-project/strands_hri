#!/usr/bin/env python

import rospy
import actionlib
import dynamic_reconfigure.client
from strands_perception_people_msgs.msg import PedestrianLocations
import move_base_msgs.msg
import thread
import actionlib_msgs.msg

class DynamicVelocityReconfigure():
    "A calss to reconfigure the velocity of the DWAPlannerROS."

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        sub_topic = rospy.get_param("~pedestrian_locations", '/pedestrian_localisation/localisations')
        rospy.Subscriber(sub_topic, PedestrianLocations, self.pedestrianCallback, None, 5)
        self.fast = True
        rospy.loginfo("Creating dynamic reconfigure client")
        self.client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
        rospy.loginfo(" ...done")
        rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient(
            'move_base',
            move_base_msgs.msg.MoveBaseAction
        )
        self.baseClient.wait_for_server()
        rospy.loginfo("...done")
        rospy.loginfo("Reading move_base parameters")
        self.getCurrentSettings()
        rospy.loginfo("Reading parameters")
        self.threshold = rospy.get_param(name+"/timeout",2.0)
        current_time = rospy.get_time()
        self.timeout = current_time + self.threshold
        self.max_dist = rospy.get_param(name+"/max_dist",5.0)
        self.min_dist = rospy.get_param(name+"/min_dist",1.2)
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, execute_cb = None, auto_start = False)
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

    def getCurrentSettings(self):
        max_vel_x = round(rospy.get_param("/move_base/DWAPlannerROS/max_vel_x"), 2)
        max_trans_vel = round(rospy.get_param("/move_base/DWAPlannerROS/max_trans_vel"),2)
        max_rot_vel = round(rospy.get_param("/move_base/DWAPlannerROS/max_rot_vel"), 2)
        min_vel_x = round(rospy.get_param("/move_base/DWAPlannerROS/min_vel_x"), 2)
        min_trans_vel = round(rospy.get_param("/move_base/DWAPlannerROS/min_trans_vel"),2)
        min_rot_vel = round(rospy.get_param("/move_base/DWAPlannerROS/min_rot_vel"), 2)
        self.fast_param = {
            'max_vel_x' : max_vel_x,
            'max_trans_vel' : max_trans_vel,
            'max_rot_vel' : max_rot_vel,
            'min_vel_x' : min_vel_x,
            'min_trans_vel' : min_trans_vel,
            'min_rot_vel' : min_rot_vel
        }
        #rospy.loginfo("Found following default values for move_base: %s", self.fast_param)

    def resetSpeed(self):
        rospy.logdebug("Resetting speeds to max:")
        rospy.logdebug(" Setting parameters: %s", self.fast_param)
        self.client.update_configuration(self.fast_param)

    def pedestrianCallback(self, pl):
        if not self._as.is_active():
            rospy.logdebug("No active goal")
            return

        if len(pl.poses) > 0:
            rospy.logdebug("Found pedestrian: ")
            rospy.logdebug(" Pedestrian distance: %s", pl.min_distance)
            factor = pl.min_distance - self.min_dist
            factor = factor if factor > 0.0 else 0.0
            factor /= (self.max_dist - self.min_dist)
            factor = 1.0 if factor > 1.0 else factor
            trans_speed = factor * self.fast_param['max_vel_x']
            trans_speed = round(trans_speed, 2)
            rospy.logdebug("Calculated translational speed: %s", trans_speed)
            rot_speed = factor * self.fast_param['max_rot_vel']
            rot_speed = round(rot_speed, 2)
            rospy.logdebug("Calculated rotaional speed: %s", rot_speed)
            if not trans_speed == self.fast_param['max_vel_x'] and not rot_speed == self.fast_param['max_rot_vel']:
                self.slow_param = {'max_vel_x' : trans_speed, 'max_trans_vel' : trans_speed, 'max_rot_vel' : rot_speed}
                self.client.update_configuration(self.slow_param)
                rospy.logdebug(" Setting parameters: %s", self.slow_param)
                self.fast = False
            self.timeout = rospy.get_time() + self.threshold
        elif rospy.get_time() > self.timeout:
            rospy.logdebug("Not found any pedestrians:")
            if not self.fast:
                self.resetSpeed()
                self.fast = True
            else:
                rospy.logdebug(" Already fast")

    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        self.getCurrentSettings()
        rospy.logdebug("Received goal:\n%s", self._goal)
        self.resetSpeed()
        thread.start_new_thread(self.moveBaseThread,(self._goal,))

    def preemptCallback(self):
        rospy.logdebug("Cancelled execution of goal:\n%s", self._goal)
        self.baseClient.cancel_all_goals()
        self._as.set_preempted()
        self.resetSpeed()

    def moveBaseThread(self,goal):
        ret = self.moveBase(goal)
        self.resetSpeed()
        if not self._as.is_preempt_requested() and ret:
            self._as.set_succeeded(self.result)
        elif not self._as.is_preempt_requested() and not ret:
            self._as.set_aborted(self.result)

    def moveBase(self,goal):
        rospy.logdebug('Moving robot to goal: %s', goal)
        self.baseClient.send_goal(goal, feedback_cb=self.moveBaseFeedbackCallback)
        self.baseClient.wait_for_result()
        self.result = self.baseClient.get_result()
        if self.baseClient.get_state() != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            return False

        rospy.sleep(rospy.Duration.from_sec(0.3)) #avoid jumping out of a state immediately after entering it - actionlib bug
        return True

    def moveBaseFeedbackCallback(self,fb):
       self._as.publish_feedback(fb)

if __name__ == '__main__':
    rospy.init_node("human_aware_navigation")
    dvr = DynamicVelocityReconfigure(rospy.get_name())
    rospy.spin()
