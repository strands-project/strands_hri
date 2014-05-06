#! /usr/bin/env python

import rospy
import actionlib
import strands_interaction_behaviours.msg
import actionlib_msgs.msg
import flir_pantilt_d46.msg

class IdleBehaviour(object):
# create messages that are used to publish feedback/result
    _feedback = strands_interaction_behaviours.msg.BehaviourSwitchFeedback()

    def __init__(self, name):
        # Variables
        self._action_name = name
        self.mode = 0

        #Getting parameters
        self.runtime = rospy.get_param(self._action_name + "/runtime", 0)

        # BehaviourSwitch client
        rospy.loginfo("%s: Creating behaviour_switch client", name)
        self.bsClient = actionlib.SimpleActionClient('behaviour_switch', strands_interaction_behaviours.msg.BehaviourSwitchAction)
        self.bsClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # PTU client
        rospy.loginfo("%s: Creating PTU client", name)
        self.ptuClient = actionlib.SimpleActionClient('SetPTUState', flir_pantilt_d46.msg.PtuGotoAction)
        self.ptuClient.wait_for_server()
        rospy.loginfo("%s: ...done", name)

        # Starting server
        rospy.loginfo("%s: Starting action server", name)
        self._as = actionlib.SimpleActionServer(self._action_name, strands_interaction_behaviours.msg.IdleBehaviourAction, execute_cb=self.exCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        self._as.start()
        rospy.loginfo("%s: ...done.", name)


    def exCallback(self, goal):
        #self.turnPTU() #TODO: Need to figure out the problem with turning the PTU first
        idle_goal = strands_interaction_behaviours.msg.BehaviourSwitchGoal()
        idle_goal.runtime_seconds = self.runtime
        self.bsClient.send_goal(idle_goal)
        self.bsClient.wait_for_result()
        if self.bsClient.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            self._as.set_succeeded()
        elif not self._as.is_preempt_requested():
            self._as.set_aborted()

    def preemptCallback(self):
        self.bsClient.cancel_all_goals()
        self._as.set_preempted()

    def turnPTU(self):
        goal = flir_pantilt_d46.msg.PtuGotoGoal()
        goal.pan = -180
        goal.tilt = 0
        goal.pan_vel = 10
        goal.tilt_vel = 0
        self.ptuClient.send_goal_and_wait(goal)

if __name__ == '__main__':
    rospy.init_node('idle_behaviour')
    IdleBehaviour(rospy.get_name())
    rospy.spin()

