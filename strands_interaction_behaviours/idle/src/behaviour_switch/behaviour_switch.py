#! /usr/bin/env python

import rospy
import actionlib
import strands_interaction_behaviours.msg
import strands_gazing.msg
import geometry_msgs.msg

import thread

class BehaviourSwitch(object):
# create messages that are used to publish feedback/result
    _feedback = strands_interaction_behaviours.msg.BehaviourSwitchFeedback()

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.mode = 0

        # Getting parameters
        people_array_topic = rospy.get_param("~people_array_topic", '/upper_body_detector/bounding_box_centres')
        self.people_closest_topic = rospy.get_param("~people_closest_topic", '/upper_body_detector/closest_bounding_box_centre')

        # Gaze client
        rospy.loginfo("%s: Creating gaze client", name)
        self.gazeClient = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)
        self.gazeClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Idle client
        rospy.loginfo("%s: Creating idle client", name)
        self.idleClient = actionlib.SimpleActionClient('idle_server', strands_interaction_behaviours.msg.InteractionIdleAction)
        self.idleClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(self._action_name, strands_interaction_behaviours.msg.BehaviourSwitchAction, execute_cb=None, auto_start = False)
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # Create Subscribers
        rospy.Subscriber(people_array_topic, geometry_msgs.msg.PoseArray, self.peopleCallback, None, 10)


    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        rospy.loginfo("Received goal:\n%s",self._goal)
        current_time = rospy.get_time()
        self.end_time = current_time + self._goal.runtime_seconds if self._goal.runtime_seconds > 0 else -1.0
        if self.end_time != -1.0:
            thread.start_new_thread(self.checkTime,())

    def preemptCallback(self):
        rospy.loginfo("Preempting current goal:\n%s",self._goal)
        self.idleClient.cancel_all_goals()
        self.gazeClient.cancel_all_goals()
        self._as.set_preempted()

    def peopleCallback(self, pl):
        if not self._as.is_active():
            return

        self._feedback.remaining_runtime = self.end_time - rospy.get_time() if self.end_time > 0 else -1
        if len(pl.poses) == 0 and self.mode != 1:
            goal = strands_interaction_behaviours.msg.InteractionIdleGoal
            goal.runtime_seconds = 0
            print 'goal:', goal
            self.idleClient.send_goal(goal)
            self._feedback.person_found = False
            self.mode = 1
        elif len(pl.poses) > 0 and self.mode != -1:
            self.idleClient.cancel_all_goals()
            goal = strands_gazing.msg.GazeAtPoseGoal
            goal.topic_name = self.people_closest_topic
            goal.runtime_sec = 0
            print 'goal:', goal
            self.gazeClient.send_goal(goal)
            self._feedback.person_found = True
            self.mode = -1
        self._as.publish_feedback(self._feedback)
        rospy.sleep(1)

    def checkTime(self):
        while self._as.is_active():
            if rospy.get_time() > self.end_time and self.end_time > 0:
                rospy.loginfo("Execution time has been reached. Goal terminated successfully")
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded()



if __name__ == '__main__':
    rospy.init_node('behaviour_switch')
    BehaviourSwitch(rospy.get_name())
    rospy.spin()

