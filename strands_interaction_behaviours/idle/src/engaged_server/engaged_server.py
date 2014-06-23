#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import strands_interaction_behaviours.msg
import ros_mary_tts.msg
from ros_datacentre.message_store import MessageStoreProxy
import std_msgs.msg
from random import randint


class EngagedServer(object):
# create messages that are used to publish feedback/result
    _feedback = strands_interaction_behaviours.msg.BehaviourSwitchFeedback()

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.mode = 0

        #Getting parameters
        self.runtime = rospy.get_param("~runtime", 0)
        dialogue_option = rospy.get_param("~dialogue_option", "engaged")
        speaktopic = rospy.get_param("~text", '/nhm/speak')

        self.speak_pub = rospy.Publisher(speaktopic, std_msgs.msg.String, latch=True)

        # Mary client
        rospy.loginfo("%s: Creating mary client", name)
        self.maryClient = actionlib.SimpleActionClient(
            'speak',
            ros_mary_tts.msg.maryttsAction
        )
        self.maryClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            strands_interaction_behaviours.msg.InteractionEngagedAction,
            execute_cb=None,
            auto_start=False
        )
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)

        # Loading sentences
        self.sentences = self.loadDialogue(dialogue_option)

    def loadDialogue(self, dialogue_name):
        msg_store = MessageStoreProxy(collection="hri_behaviours")
        query_meta = {}
        query_meta["hri_dialogue"] = dialogue_name
        if len(msg_store.query(std_msgs.msg.String._type, {}, query_meta)) == 0:
            rospy.logerr("Desired dialogue options '"+dialogue_name+"' not in datacentre")
            raise Exception("Can't find dialogue.")
        else:
            message_list = msg_store.query(std_msgs.msg.String._type, {}, query_meta)
            sentences = []
            for message, meta in message_list:
                sentences.append(str(message.data))
            return sentences

    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        rospy.logdebug("Received goal:\n%s", self._goal)
        self.speak()

    def preemptCallback(self):
        rospy.loginfo("Action is being preempted")
        self.maryClient.cancel_all_goals()
        self._as.set_preempted()

    def speak(self):
        rospy.logdebug("Speak")
        mary_goal = ros_mary_tts.msg.maryttsGoal
        sentence = self.sentences[randint(0, len(self.sentences)-1)]
        self.speak_pub.publish(sentence)
        mary_goal.text = sentence
        self.maryClient.send_goal(mary_goal)
        self.maryClient.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('engaged_server')
    EngagedServer(rospy.get_name())
    rospy.spin()
