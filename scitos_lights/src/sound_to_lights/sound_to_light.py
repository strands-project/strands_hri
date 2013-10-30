#!/usr/bin/env python

import rospy
import actionlib
import scitos_lights.msg
import thread

from peak_detect.peak_detect import PeakMonitor
from scitos_msgs.msg import HeadLightState

class SoundLights():
    "A class to make the lights on the robot head move according to the output level of the sound card."

    # Create feedback and result messages
    _feedback = scitos_lights.msg.SoundLightsFeedback()
    _result   = scitos_lights.msg.SoundLightsResult()

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, scitos_lights.msg.SoundLightsAction, execute_cb = None, auto_start = False)
        self._as.register_goal_callback(self.goalCallback)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        pub_topic = rospy.get_param("~cmd_head_light", '/head/cmd_light_state')
        self.pub = rospy.Publisher(pub_topic, HeadLightState)
        self.sink_name = rospy.get_param("~sink_name", 'alsa_output.usb-Burr-Brown_from_TI_USB_Audio_CODEC-00-CODEC.analog-stereo')
        self.meter_rate = 344
        self.max_sample_value = 127
        self.display_scale = 2
        self.max_spaces = self.max_sample_value >> self.display_scale
        self.rate = 0.2
        self.last = rospy.get_time()

    def goalCallback(self):
        self._goal = self._as.accept_new_goal()
        rospy.loginfo("Received goal:\n%s", self._goal)
        current_time = rospy.get_time()
        self.end_time = current_time + self._goal.seconds if self._goal.seconds > 0 else -1.0
        #turn off lights

    def preemptCallback(self):
        rospy.loginfo("Cancelled execution of goal:\n%s", self._goal)
        #reset lights to rotate
        self._result.expired = False
        self._as.set_preempted(self._result)

    def switchLEDs(self, numLEDs):
        lights = HeadLightState()
        numLEDs = int(numLEDs)
        lights.HeadLightInterval = 0
        lights.LEDState0 = 1 if numLEDs > 0 else 0
        lights.LEDState1 = 1 if numLEDs > 1 else 0
        lights.LEDState2 = 1 if numLEDs > 2 else 0
        lights.LEDState3 = 1 if numLEDs > 3 else 0
        lights.LEDState4 = 1 if numLEDs > 3 else 0
        lights.LEDState5 = 1 if numLEDs > 2 else 0
        lights.LEDState6 = 1 if numLEDs > 1 else 0
        lights.LEDState7 = 1 if numLEDs > 0 else 0
        lights.LEDPhase0 = 1
        lights.LEDPhase1 = 1
        lights.LEDPhase2 = 1
        lights.LEDPhase3 = 1
        lights.LEDPhase4 = 1
        lights.LEDPhase5 = 1
        lights.LEDPhase6 = 1
        lights.LEDPhase7 = 1
        lights.LEDAmplitude0 = 1.0
        lights.LEDAmplitude1 = 1.0
        lights.LEDAmplitude2 = 1.0
        lights.LEDAmplitude3 = 1.0
        lights.LEDAmplitude4 = 1.0
        lights.LEDAmplitude5 = 1.0
        lights.LEDAmplitude6 = 1.0
        lights.LEDAmplitude7 = 1.0
        if not rospy.is_shutdown():
            if self.last + self.rate < rospy.get_time():
                self.pub.publish(lights)
                self.last = rospy.get_time()

    def grabAudioLevel(self):
        monitor = PeakMonitor(self.sink_name, self.meter_rate)
        for sample in monitor:
            lights = float(sample) / float(self.max_sample_value) * 20
            sample = sample >> self.display_scale
            bar = '>' * sample
            spaces = ' ' * (self.max_spaces - sample)
            print ' %3d %s%s\r' % (sample, bar, spaces),
            self.switchLEDs(lights)


if __name__ == '__main__':
    rospy.init_node("sound_to_light")
    sl = SoundLights(rospy.get_name())
    thread.start_new_thread(sl.grabAudioLevel,())
    #sl.grabAudioLevel()
    rospy.spin()
