#!/usr/bin/env python

import rospy
import actionlib

import strands_hri_utils.msg
import ros_mary_tts.msg
import strands_visualise_speech.msg

class VisualSpeechServer(  ):
    """Convenience action server to use mary and the head light control with just one call"""
    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, strands_hri_utils.msg.VisualSpeechAction, execute_cb = self.exCb, auto_start = False)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        rospy.loginfo("Connecting to mary server")
        self.mary_client = actionlib.SimpleActionClient('speak', ros_mary_tts.msg.maryttsAction)
        self.mary_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.loginfo("Connecting to head light server")
        self.light_client = actionlib.SimpleActionClient('sound_to_light', strands_visualise_speech.msg.SoundLightsAction)
        self.light_client.wait_for_server()
        rospy.loginfo(" ...done")


    def exCb(self, goal):
        print goal
        # Start lights
        lights = strands_visualise_speech.msg.SoundLightsGoal()
        lights.seconds = 0
        self.light_client.send_goal(lights)
        text = ros_mary_tts.msg.maryttsGoal()
        text.text = goal.text
        rospy.sleep(1.0)
        self.mary_client.send_goal(text)
        self.mary_client.wait_for_result()
        rospy.sleep(1.0)
        self.light_client.cancel_all_goals()
        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node("visual_speech")
    vs = VisualSpeechServer(rospy.get_name())
    rospy.spin()

