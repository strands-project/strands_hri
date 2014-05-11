#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import strands_interaction_behaviours.msg
import std_msgs
import geometry_msgs.msg
import ros_mary_tts.msg
import strands_gazing.msg
from ros_datacentre.message_store import MessageStoreProxy

import thread
from random import randint
from random import uniform

class IdleServer(object):
    # create messages that are used to publish feedback/result
    _feedback = strands_interaction_behaviours.msg.InteractionIdleFeedback()
    _result   = strands_interaction_behaviours.msg.InteractionIdleResult()
    def __init__(self, name):
        # Variables
        self._action_name = name
        self.look_trigger = 0
        self.speak_trigger = 0
        self.seq = 1

        # Getting parameters
        self.head_topic = rospy.get_param("~head_pose", '/idle_server/head_pose')
        dialogue_option = rospy.get_param("~dialogue_option","idle_gameplay")
        self.display_no = rospy.get_param("~display", 0)

        # Publishers and subscribers
        self.pose_pub = rospy.Publisher(self.head_topic,geometry_msgs.msg.PoseStamped)

        # Mary client
        rospy.loginfo("%s: Creating mary client", name)
        self.maryClient = actionlib.SimpleActionClient('speak', ros_mary_tts.msg.maryttsAction)
        self.maryClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Gaze client
        rospy.loginfo("%s: Creating gaze client", name)
        self.gazeClient = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)
        self.gazeClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(self._action_name, strands_interaction_behaviours.msg.InteractionIdleAction, execute_cb=None, auto_start = False)
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # Loading sentences
        self.sentences = self.loadDialogue(dialogue_option)
        #print self.sentences

    def loadDialogue(self, dialogue_name): #TODO: Use dynamic reconfigure?!
        msg_store = MessageStoreProxy(collection="hri_behaviours")

        query_meta = {}
        query_meta["hri_dialogue"] = dialogue_name
        if len(msg_store.query(std_msgs.msg.String._type, {}, query_meta)) == 0:
            rospy.logerr("Desired dialogue options'"+dialogue_name+"' not in datacentre")
            raise Exception("Can't find dialogue.")

        else :
            message_list = msg_store.query(std_msgs.msg.String._type, {}, query_meta)

            sentences = []
            for message, meta in message_list:
                sentences.append(str(message.data))
                #rospy.loginfo(message.data)

            return sentences

    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        self.loop = True
        current_time = rospy.get_time()
        self.end_time = current_time + self._goal.runtime_seconds if self._goal.runtime_seconds > 0 else -1.0
        gaze_goal = strands_gazing.msg.GazeAtPoseGoal
        gaze_goal.runtime_sec = 0
        gaze_goal.topic_name = self.head_topic
        self.gazeClient.send_goal(gaze_goal)
        thread.start_new_thread(self.idle_behaviour, ())

    def preemptCallback(self):
        rospy.logdebug("Cancelled execution of goal.")
        self.loop = False
        self._as.set_preempted()

    def look(self):
        rospy.logdebug("Execute: Look around")
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = '/head_xtion_depth_optical_frame'
        pose.header.stamp = rospy.Time.now()
        pose.header.seq = self.seq
        self.seq += 1
        pose.pose.position.x = uniform(2.5,3.5)
        pose.pose.position.y = uniform(0.28,0.32)
        pose.pose.position.z = 5.0
        pose.pose.orientation.w = 1
        if self._as.is_active():
            self.pose_pub.publish(pose)
        rospy.sleep(3)
        pose.header.stamp = rospy.Time.now()
        pose.header.seq = self.seq
        self.seq += 1
        pose.pose.position.x = -uniform(2.5,3.5)
        pose.pose.position.y = uniform(0.28,0.32)
        pose.pose.position.z = 5.0
        pose.pose.orientation.w = 1
        if self._as.is_active():
            self.pose_pub.publish(pose)
        rospy.sleep(3)
        pose.header.stamp = rospy.Time.now()
        pose.header.seq = self.seq
        self.seq += 1
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.3
        pose.pose.position.z = 5.0
        pose.pose.orientation.w = 1
        if self._as.is_active():
            self.pose_pub.publish(pose)
        rospy.sleep(3)

    def speak(self):
        rospy.logdebug("Speak")
        mary_goal = ros_mary_tts.msg.maryttsGoal
        sentence = self.sentences[randint(0, len(self.sentences)-1)]
        mary_goal.text = sentence
        self.maryClient.send_goal(mary_goal)
        self.maryClient.wait_for_result()


    def idle_behaviour(self):
        while self.loop:
            self._feedback.remaining_runtime = self.end_time - rospy.get_time() if self.end_time > 0 else -1
            if self.look_trigger == 0:
                thread.start_new_thread(self.look,())
                self.look_trigger = randint(10,20)
            self.look_trigger -= 1
            self._feedback.next_look = self.look_trigger
            if self.speak_trigger == 0:
                thread.start_new_thread(self.speak,())
                self.speak_trigger = randint(60,120)
            self.speak_trigger -= 1
            self._feedback.next_speak = self.speak_trigger
            self._as.publish_feedback(self._feedback)
            rospy.sleep(1)
            if rospy.get_time() > self.end_time and self.end_time > 0:
                rospy.logdebug("Execution time has been reached. Goal terminated successfully")
                rospy.logdebug('%s: Succeeded' % self._action_name)
                self._as.set_succeeded()
                break


if __name__ == '__main__':
    rospy.init_node('idle_server')
    IdleServer(rospy.get_name())
    rospy.spin()
