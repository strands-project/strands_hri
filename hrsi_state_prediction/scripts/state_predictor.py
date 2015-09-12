#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 16:57:54 2015

@author: cdondrup
"""

import rospy
from hrsi_representation.msg import QTCArray
from hrsi_state_prediction.simple_model import QTCBPassBy, QTCCPassBy, QTCBCPassBy, QTCBPathCrossing, QTCBCPathCrossing
from hrsi_state_prediction.particle_filter import ParticleFilterPredictor
from hrsi_state_prediction.msg import QTCPrediction, QTCPredictionArray
from dynamic_reconfigure.server import Server as DynServer
from hrsi_state_prediction.cfg import StatePredictorConfig
import json
import time


class StatePredictor(object):
    __interaction_types = ["passby", "pathcrossing"]

    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.dyn_srv = DynServer(StatePredictorConfig, self.dyn_callback)
        self.pf = ParticleFilterPredictor(
            paths=[
                '/home/cdondrup/PhD/shaping_hrsi/hmms/Bristol/left/qtcbc_4_nc.hmm',
                '/home/cdondrup/PhD/shaping_hrsi/hmms/Bristol/right/qtcbc_4_nc.hmm',
                '/home/cdondrup/PhD/shaping_hrsi/hmms/Bristol/qtcbc_empty_uniform.hmm'
            ]
        )
        self.sm = {
            "passby": {
                "qtcbs": QTCBPassBy(),
                "qtccs": QTCCPassBy(),
                "qtcbcs": QTCBCPassBy(),
                "qtcbcs_argprobd": QTCBCPassBy()
            },
            "pathcrossing": {
                "qtcbs": QTCBPathCrossing(),
                "qtcbcs": QTCBCPathCrossing()
            }
        }
        self.pub = rospy.Publisher("~prediction_array", QTCPredictionArray, queue_size=10)
        rospy.Subscriber(rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array"), QTCArray, self.callback, queue_size=1)
        rospy.loginfo("... all done")

    def dyn_callback(self, config, level):
        self.prior = self.__interaction_types[config["type"]]
        rospy.loginfo("Interaction type set to %s" % self.prior)
        return config

    def callback(self, msg):
        interaction_type = self.prior # No classification yet
        start = time.time()
        out = QTCPredictionArray()
        out.header = msg.header
        for q in msg.qtc:
            m = QTCPrediction()
            m.uuid = q.uuid
            try:
                qtc = json.loads(q.qtc_serialised)
                dist = json.loads(q.prob_distance_serialised)[-1]
                prediction = self.sm[interaction_type][q.qtc_type].predict(
                    qtc[-1],
                    dist
                )
#                prediction = self.pf.predict(m.uuid, qtc, dist)
#                self.pf.predict(m.uuid, qtc)
            except KeyError:
                print q.qtc_type + " not defined"
                return
            print "~~~~~~~~~~", prediction
            m.qtc_serialised = json.dumps(prediction)
            out.qtc.append(m)
        self.pub.publish(out)
#        print "elapsed:", time.time() - start


if __name__ == "__main__":
    rospy.init_node("qtc_state_predictor")
    s = StatePredictor(rospy.get_name())
    rospy.spin()