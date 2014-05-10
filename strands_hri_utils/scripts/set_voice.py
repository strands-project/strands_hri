#! /usr/bin/env python

import rospy
import ros_mary_tts.srv

class IdleBehaviour(object):
# create messages that are used to publish feedback/result

    def __init__(self, name):
        # Variables
        self._action_name = name

        #Getting parameters
        self.locale = rospy.get_param("~speak_locale", "de")
        self.voice = rospy.get_param("~speak_voice", "dfki-pavoque-neutral-hsmm")

        self.setVoice()

    def setVoice(self):
        rospy.wait_for_service('ros_mary/set_locale')
        rospy.wait_for_service('ros_mary/set_voice')
        try:
            set_locale = rospy.ServiceProxy('ros_mary/set_locale', ros_mary_tts.srv.SetLocale)
            set_voice  = rospy.ServiceProxy('ros_mary/set_voice', ros_mary_tts.srv.SetVoice)
            set_locale(self.locale)
            set_voice(self.voice)
            return
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s",e)

if __name__ == '__main__':
    rospy.init_node('set_voice')
    IdleBehaviour(rospy.get_name())
    rospy.spin()

