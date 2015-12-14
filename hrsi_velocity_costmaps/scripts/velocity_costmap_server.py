#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Mon Jul 27 16:48:09 2015

@author: cdondrup
"""

import rospy
from message_filters import Subscriber, TimeSynchronizer
from hrsi_state_prediction.msg import QTCPredictionArray
from bayes_people_tracker.msg import PeopleTracker
import json
from hrsi_velocity_costmaps.costmap_creator import CostmapCreator
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from dynamic_reconfigure.server import Server as DynServer
from hrsi_velocity_costmaps.cfg import VelocityCostmapsConfig
from tf import TransformListener


class VelocityCostmapServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s..." % name)
        self.tf = TransformListener()
        self.cc = CostmapCreator(
            rospy.Publisher("/velocity_costmap", OccupancyGrid, queue_size=10, latch=True),
            rospy.Publisher("~origin", PoseStamped, queue_size=10)
        )
        self.dyn_srv = DynServer(VelocityCostmapsConfig, self.dyn_callback)
        subs = [
            Subscriber(rospy.get_param("~qtc_topic", "/qtc_state_predictor/prediction_array"), QTCPredictionArray),
            Subscriber(rospy.get_param("~ppl_topic", "/people_tracker/positions"), PeopleTracker)
        ]
        ts = TimeSynchronizer(
            fs=subs,
            queue_size=60
        )
        ts.registerCallback(self.callback)
        rospy.loginfo("... all done.")

    def dyn_callback(self, config, level):
        self.cc.resolution = config["costmap_resolution"]
        self.cc.min_costs  = config["min_costs"]
        self.cc.max_costs  = config["max_costs"]
        return config

    def callback(self, qtc, ppl):
        vels = []
        try:
            t = self.tf.getLatestCommonTime("base_link", ppl.header.frame_id)
            vs = Vector3Stamped(header=ppl.header)
            vs.header.stamp = t
            for v in ppl.velocities:
                vs.vector = v
                vels.append(self.tf.transformVector3("base_link", vs).vector)
        except Exception as e:
            rospy.logwarn(e)
            return
        data_buffer = {
            e.uuid: {
                "qtc": json.loads(e.qtc_serialised),
                "angle": ppl.angles[ppl.uuids.index(e.uuid)],
                "velocity": vels[ppl.uuids.index(e.uuid)]
            } for e in qtc.qtc
        }
        element = data_buffer[ppl.uuids[ppl.distances.index(ppl.min_distance)]] # Only taking the closest person for now
        qtc = element["qtc"].split(',')
        self.cc.publish(
            angle=element["angle"],
            qtc_symbol=','.join([qtc[0]] if len(qtc) == 2 else [qtc[0], qtc[2]]),
            velocity=element["velocity"]
        )

if __name__ == "__main__":
    rospy.init_node("velocity_costmap_server")
    v = VelocityCostmapServer(rospy.get_name())
    rospy.spin()