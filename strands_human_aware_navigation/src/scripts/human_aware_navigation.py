#!/usr/bin/env python

import rospy
import actionlib
import dynamic_reconfigure.client
from strands_perception_people_msgs.msg import PedestrianLocations
import strands_perception_people_msgs.srv
import move_base_msgs.msg
import thread
import actionlib_msgs.msg
import strands_gazing.msg


class DynamicVelocityReconfigure():
    "A calss to reconfigure the velocity of the DWAPlannerROS."

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
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
        rospy.loginfo("Creating gaze client.")
        self.gazeClient = actionlib.SimpleActionClient(
            'gaze_at_pose',
            strands_gazing.msg.GazeAtPoseAction
        )
        self.gazeClient.wait_for_server()
        rospy.loginfo("...done")
        rospy.loginfo("Reading move_base parameters")
        self.getCurrentSettings()
        rospy.loginfo("Reading parameters")
        self.threshold = rospy.get_param(name+"/timeout", 4.0)
        current_time = rospy.get_time()
        self.timeout = current_time + self.threshold
        self.max_dist = rospy.get_param(name+"/max_dist", 5.0)
        self.min_dist = rospy.get_param(name+"/min_dist", 1.5)
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, move_base_msgs.msg.MoveBaseAction, self.goalCallback, auto_start = False)
        #self._as.register_goal_callback()
        #self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        sub_topic = rospy.get_param("~pedestrian_locations", '/pedestrian_localisation/localisations')
        rospy.Subscriber(sub_topic, PedestrianLocations, self.pedestrianCallback, None, 5)

        self.last_cancel_time=rospy.Time(0)
        rospy.Subscriber("/human_aware_navigation/cancel" , actionlib_msgs.msg.GoalID, self.cancel_time_checker_cb)

    def cancel_time_checker_cb(self,msg):
        self.last_cancel_time=rospy.get_rostime()

    def getCurrentSettings(self):
        max_vel_x = 0.55 #round(rospy.get_param("/move_base/DWAPlannerROS/max_vel_x"), 2)
        max_trans_vel = 0.55 #round(rospy.get_param("/move_base/DWAPlannerROS/max_trans_vel"),2)
        max_rot_vel = 1.0 #round(rospy.get_param("/move_base/DWAPlannerROS/max_rot_vel"), 2)
        min_vel_x = 0.0 #round(rospy.get_param("/move_base/DWAPlannerROS/min_vel_x"), 2)
        min_trans_vel = 0.1 #round(rospy.get_param("/move_base/DWAPlannerROS/min_trans_vel"),2)
        min_rot_vel = 0.4 #round(rospy.get_param("/move_base/DWAPlannerROS/min_rot_vel"), 2)
        self.fast_param = {
            'max_vel_x': max_vel_x,
            'max_trans_vel': max_trans_vel,
            'max_rot_vel': max_rot_vel,
            'min_vel_x': min_vel_x,
            'min_trans_vel': min_trans_vel,
            'min_rot_vel': min_rot_vel
        }
        #rospy.loginfo("Found following default values for move_base: %s", self.fast_param)

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
                    self.gazeClient.cancel_all_goals()
                    self._as.set_aborted()

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
            if not trans_speed == self.fast_param['max_vel_x']:  # and not rot_speed == self.fast_param['max_rot_vel']:
                gaze_goal = strands_gazing.msg.GazeAtPoseGoal()
                gaze_goal.runtime_sec = 0
                gaze_goal.topic_name = "/upper_body_detector/closest_bounding_box_centre"
                self.gazeClient.send_goal(gaze_goal)
                self.slow_param = {'max_vel_x': trans_speed, 'max_trans_vel': trans_speed}  #, 'max_rot_vel' : rot_speed}
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
                gaze_goal = strands_gazing.msg.GazeAtPoseGoal()
                gaze_goal.runtime_sec = 0
                gaze_goal.topic_name = "/pose_extractor/pose"
                self.gazeClient.send_goal(gaze_goal)
                self.resetSpeed()
                self.fast = True
            else:
                rospy.logdebug(" Already fast")

    def goalCallback(self,goal):
        #self._goal = self._as.accept_new_goal()
        self._goal=goal
        gaze_goal = strands_gazing.msg.GazeAtPoseGoal()
        gaze_goal.runtime_sec = 0
        gaze_goal.topic_name = "/pose_extractor/pose"
        self.gazeClient.send_goal(gaze_goal)
        self.getCurrentSettings()
        rospy.logdebug("Received goal:\n%s", self._goal)
        self.resetSpeed()
        #thread.start_new_thread(self.moveBaseThread,(self._goal,))
        try:
            s = rospy.ServiceProxy('/start_head_analysis', strands_perception_people_msgs.srv.StartHeadAnalysis)
            s()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        self.moveBaseThread(self._goal)

    def preemptCallback(self):
        if rospy.get_rostime()-self.last_cancel_time < rospy.Duration(1):
            rospy.logdebug("Cancelled execution of goal:\n%s", self._goal)
            self.baseClient.cancel_all_goals()
            self.gazeClient.cancel_all_goals()
            self.resetSpeed()
        self._as.set_preempted()

    def moveBaseThread(self,goal):
        ret = self.moveBase(goal)
        self.resetSpeed()
        self.gazeClient.cancel_all_goals()
        try:
            s = rospy.ServiceProxy('/stop_head_analysis', strands_perception_people_msgs.srv.StartHeadAnalysis)
            s()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        if not self._as.is_preempt_requested() and ret:
            self._as.set_succeeded(self.result)
        elif not self._as.is_preempt_requested() and not ret:
            self._as.set_aborted(self.result)

    def moveBase(self,goal):
        rospy.logdebug('Moving robot to goal: %s', goal)
        self.baseClient.send_goal(goal, feedback_cb=self.moveBaseFeedbackCallback)
        status= self.baseClient.get_state()
        while status==actionlib_msgs.msg.GoalStatus.PENDING or status==actionlib_msgs.msg.GoalStatus.ACTIVE:
            status= self.baseClient.get_state()
            self.baseClient.wait_for_result(rospy.Duration(0.2))
            self.result = self.baseClient.get_result()
            if self._as.is_preempt_requested():
                self.preemptCallback()
                break

        if self.baseClient.get_state() != actionlib_msgs.msg.GoalStatus.SUCCEEDED and self.baseClient.get_state() != actionlib_msgs.msg.GoalStatus.PREEMPTED:
            return False

        rospy.sleep(rospy.Duration.from_sec(0.3)) #avoid jumping out of a state immediately after entering it - actionlib bug
        return True

    def moveBaseFeedbackCallback(self,fb):
       self._as.publish_feedback(fb)

if __name__ == '__main__':
    rospy.init_node("human_aware_navigation")
    dvr = DynamicVelocityReconfigure(rospy.get_name())
    rospy.spin()
