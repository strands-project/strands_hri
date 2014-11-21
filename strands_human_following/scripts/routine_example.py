#!/usr/bin/env python

import rospy

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task

from datetime import time
from dateutil.tz import tzlocal

from routine_behaviours.robot_routine import RobotRoutine


class FollowingRoutine(RobotRoutine):

    def __init__(self, start, end):
        RobotRoutine.__init__(self, start, end,
                              idle_duration=rospy.Duration(5))

        self.waypoint = 'WayPoint2'

        self.start = start
        self.end = end

        rospy.sleep(0.5)

    def create_routine(self):
        # Add task to routine where the duration
        # to follow human is 300 seconds.
        # Choose a waypoint which is the nearest one to the wait point
        # which is set in conf/default.yaml
        following_task = Task(start_node_id=self.waypoint,
                              max_duration=rospy.Duration(300),
                              action='simple_follow')

        # The argument for the action, how long
        # following action should be performed
        task_utils.add_int_argument(following_task, 270)

        rospy.loginfo("Creating strands human following routine...")

        # Example to run human following the whole day several times
        # today = (self.start, self.end)
        # self.routine.repeat_every(patrol_routine, *today, times=5)

        # Example to run human following every hour
        # (can be several times)
        self.routine.repeat_every_hour(following_task, hours=1, times=1)

        # pass the routine tasks on to the runner which handles
        # the daily instantiation of actual tasks
        self.start_routine()

    # def on_idle(self):
    #    # Called when routine is idle.
    #    rospy.loginfo("Idle for too long, send the robot to " + self.waypoint)
    #    # add task to the routine to send the robot to the waypoint
    #    self.add_tasks([Task(start_node_id=self.waypoint,
    #                         max_duration=rospy.Duration(30))])


if __name__ == '__main__':
    rospy.init_node("following_routine_example")

    # Initializing working time 8:45 to 23:45
    localtz = tzlocal()
    daily_start = time(8, 45, tzinfo=localtz)
    daily_end = time(23, 45, tzinfo=localtz)

    # creating following routine
    routine = FollowingRoutine(daily_start, daily_end)
    routine.create_routine()

    rospy.spin()
