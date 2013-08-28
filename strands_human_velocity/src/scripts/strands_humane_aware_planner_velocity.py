#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
from strands_perception_people_msgs.msg import PedestrianLocations

class DynamicVelocityReconfigure():
    "A calss to reconfigure the velocity of the DWAPlannerROS."

    def __init__(self):
        rospy.init_node("human_aware_planner_velocities")
        rospy.loginfo("Starting human_aware_planner_velocity")
        rospy.Subscriber('/pedestrian_localisation/localisations', PedestrianLocations, self.callback, None, 5)
        self.threshold = 5.0
        self.timeout = rospy.get_time() + self.threshold
        self.fast = True
        rospy.loginfo("Creating dynamic reconfigure client")
        self.client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
        self.slow_param = {'max_vel_x' : 0.1}
        self.fast_param = {'max_vel_x' : 0.55}
        rospy.loginfo(" ...done")

    def callback(self, pl):
        if len(pl.poses) > 0:
            rospy.loginfo("Found pedestrian: ")
            #if not self.slow:
            #if pl.min_distance < 2.0:
            rospy.loginfo(" Pedestrian distance: %s", pl.min_distance)
            speed = pl.min_distance - 1.5
            speed = speed if speed > 0.0 else 0.0
            speed /= 2.0
            speed = 1.0 if speed > 1.0 else speed
            speed *= 0.55
            speed = round(speed, 2)
            rospy.loginfo("Calculated speed: %s", speed)
            if not speed == 0.55:
                self.slow_param = {'max_vel_x' : speed}
                self.client.update_configuration(self.slow_param)
                rospy.loginfo(" Setting parameters: %s", self.slow_param)
                self.fast = False
            #else:
                #rospy.loginfo(" Pedestrian too far away to be a problem: %s", pl.min_distance)
            #else:
                #rospy.loginfo(" Already slow")
            self.timeout = rospy.get_time() + self.threshold
        elif rospy.get_time() > self.timeout:
            rospy.loginfo("Not found any pedestrians:")
            if not self.fast:
                rospy.loginfo(" Setting parameters: %s", self.fast_param)
                self.client.update_configuration(self.fast_param)
                self.fast = True
            else:
                rospy.loginfo(" Already fast")


if __name__ == '__main__':
    dvr = DynamicVelocityReconfigure()
    rospy.spin()
