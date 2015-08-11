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
import numpy as np
from hrsi_velocity_costmaps.costmap_creator import CostmapCreator
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from dynamic_reconfigure.server import Server as DynServer
from hrsi_velocity_costmaps.cfg import VelocityCostmapsConfig
from tf import TransformListener
import time


class VelocityCostmapServer(object):
    _state_chain = [[-1,-1], [0,-1], [0,1], [1,1]]
    _current_state = 0

    def __init__(self, name):
        rospy.loginfo("Starting %s..." % name)
        self.tf = TransformListener()
        self.cc = CostmapCreator(
            rospy.Publisher("~map", OccupancyGrid, queue_size=10, latch=True),
            rospy.Publisher("~origin", PoseStamped, queue_size=10)
        )
        self.dyn_srv = DynServer(VelocityCostmapsConfig, self.dyn_callback)
        self.qtc_states = self._create_qtc_states('b')
        print self.qtc_states
        self.transdict = self._create_transition_dict(self.qtc_states)
        subs = [
            Subscriber(rospy.get_param("~qtc_topic", "/qtc_state_predictor/prediction_array"), QTCPredictionArray),
            Subscriber(rospy.get_param("~ppl_topic", "/people_tracker/positions"), PeopleTracker)
        ]
        ts = TimeSynchronizer(
            fs=subs,
            queue_size=30
        )
        ts.registerCallback(self.callback)
        rospy.loginfo("... all done.")

    def dyn_callback(self, config, level):
        self.cc.resolution = config["costmap_resolution"]
        self.cc.min_costs  = config["min_costs"]
        self.cc.max_costs  = config["max_costs"]
        return config

    def callback(self, qtc, ppl):
        start = time.time()
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
        element = data_buffer.values()[0] # Only taking the first detection into account for now
        self.cc.publish(angle=element["angle"], qtc_symbol=[element["qtc"][0]], velocity=element["velocity"])
        #self.cc.publish(angle=element["angle"], qtc_symbol=[1,1], velocity=element["velocity"])

        print "human-robot predicted qtc: %s" % element["qtc"]
        print "total elapsed:", time.time() - start

    def _create_qtc_states(self, qtc_type):
        ret_str = []
        ret_int = []
        try:
            if qtc_type == "":
                raise AttributeError()
            elif qtc_type == 'b':
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        ret_int.append([i-2, j-2])
                        ret_str.append(str(i-2) + "," + str(j-2))
            elif qtc_type is 'c':
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        for k in xrange(1, 4):
                            for l in xrange(1, 4):
                                ret_int.append([i-2, j-2, k-2, l-2])
                                ret_str.append(str(i-2) + "," + str(j-2) + "," + str(k-2) + "," + str(l-2))
            elif qtc_type is 'bc':
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        ret_int.append([i-2, j-2, np.NaN, np.NaN])
                        ret_str.append(str(i-2) + "," + str(j-2))
                for i in xrange(1, 4):
                    for j in xrange(1, 4):
                        for k in xrange(1, 4):
                            for l in xrange(1, 4):
                                ret_int.append([i-2, j-2, k-2, l-2])
                                ret_str.append(str(i-2) + "," + str(j-2) + "," + str(k-2) + "," + str(l-2))
        except AttributeError:
            rospy.logfatal("QTC type: %s not found" % qtc_type)
            return None, None
        return [s.replace('-1','-').replace('1','+') for s in ret_str], ret_int

    def _create_transition_dict(self, qtc_tuple):
        qtc = np.array(qtc_tuple[1])

        trans = np.zeros((qtc.shape[0], qtc.shape[0]))
        for i1 in xrange(qtc.shape[0]):
            for i2 in xrange(i1+1, qtc.shape[0]):
                trans[i1, i2] = np.nanmax(np.absolute(qtc[i1]-qtc[i2])) != 2
                if trans[i1, i2] == 1:
                    for j1 in xrange(qtc.shape[1]-1):
                        for j2 in xrange(j1+1, qtc.shape[1]):
                            if sum(np.absolute(qtc[i1, [j1, j2]])) == 1 \
                                    and sum(np.absolute(qtc[i2, [j1, j2]])) == 1:
                                if np.nanmax(np.absolute(qtc[i1, [j1, j2]]-qtc[i2, [j1, j2]])) > 0 \
                                        and sum(qtc[i1, [j1, j2]]-qtc[i2, [j1,j2]]) != 1:
                                    trans[i1, i2] = 5
                                    break
                        if trans[i1, i2] != 1:
                            break
                trans[i2, i1] = trans[i1, i2]

        trans[trans != 1] = 0
#        trans += np.dot(np.eye(qtc.shape[0]), 1)

        return {k:np.array(qtc_tuple[0])[np.array(trans, dtype=bool)[i]] for i, k in enumerate(qtc_tuple[0])}


if __name__ == "__main__":
    rospy.init_node("velocity_costmap_server")
    v = VelocityCostmapServer(rospy.get_name())
    rospy.spin()