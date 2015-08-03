#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Mon Jul 27 16:48:09 2015

@author: cdondrup
"""

import rospy
from message_filters import Subscriber, TimeSynchronizer
from hrsi_representation.msg import QTCArray
from bayes_people_tracker.msg import PeopleTracker
import json
import numpy as np
from random import uniform, choice
from std_msgs.msg import String
from hrsi_prediction.costmap_creator import CostmapCreator
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class QTCStatePredictor(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s..." % name)
        self.cc = CostmapCreator(
            rospy.Publisher("~map", OccupancyGrid, queue_size=10, latch=True),
            rospy.Publisher("~origin", PoseStamped, queue_size=10),
            100,
            100
        )
        self.cnt = 0
        self.qtc_states = self._create_qtc_states('b')
        print self.qtc_states
        self.transdict = self._create_transition_dict(self.qtc_states)
        self.pub = rospy.Publisher("~prediction", String, queue_size=10)
        subs = [
            Subscriber(rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array"), QTCArray),
            Subscriber(rospy.get_param("~ppl_topic", "/people_tracker/positions"), PeopleTracker)
        ]
        ts = TimeSynchronizer(
            fs=subs,
            queue_size=30
        )
        ts.registerCallback(self.callback)
        rospy.loginfo("... all done.")

    def callback(self, qtc, ppl):
        self.cnt += 1
        data_buffer = {e.uuid: {"qtc": json.loads(e.qtc_serialised)[-1], "distance": ppl.distances[ppl.uuids.index(e.uuid)], "angle": ppl.angles[ppl.uuids.index(e.uuid)]} for e in qtc.qtc}
#        rospy.loginfo("%s: TIME_MSG: %s, BUFFER: %s" % (str(self.cnt),str(qtc.header.stamp.to_sec()),data_buffer))
#        for element in data_buffer.values():
        element = data_buffer.values()[0] # Only taking the first detection into account for now
        pred = [choice([-1,1]),element["qtc"][1]] if element["qtc"][0] == 0 else element["qtc"] # Always assume the robot moves
        if element["distance"] <= 5.0:
            if element["qtc"][0] == -1:
                x = uniform(0,1)
                prob = 2.0 / element["distance"]
                prob = prob if prob <= 1.0 else 1.0
                print "PROB: %s - %s = %s -> slow down: %s" %(x,prob,x-prob,x-prob<0.0)
                if x - prob <= 0.0:
                    pred = [0,element["qtc"][1]]
                else:
                    pred = [-1,element["qtc"][1]]
            elif element["qtc"][0] == 0:
                if element["qtc"][1] == 1:
                    pred = [1,element["qtc"][1]]
                elif element["qtc"][1] == 0 and element["distance"] > 2.0: # No one moves, take initiative
                    pred = [-1,element["qtc"][1]]
                else:
                    pred = [0,element["qtc"][1]]

        self.cc.publish(angle=element["angle"], size=np.pi/8, qtc_symbol=pred[0])

        print "human-robot distance: %s, current qtc: %s, predicted qtc: %s" % (element["distance"], element["qtc"], pred)
        self.pub.publish(self.qtc_states[0][self.qtc_states[1].index(pred)])


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
    rospy.init_node("qtc_state_predictor")
    q = QTCStatePredictor(rospy.get_name())
    rospy.spin()