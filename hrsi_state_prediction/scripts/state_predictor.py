#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 16:57:54 2015

@author: cdondrup
"""

import rospy
from hrsi_representation.msg import QTCArray
from hrsi_state_prediction.simple_model import QTCBPassBy, QTCCPassBy, QTCBCPassBy, QTCBPathCrossing
from hrsi_state_prediction.particle_filter import ParticleFilter
from hrsi_state_prediction.msg import QTCPrediction, QTCPredictionArray
import json
import time


class StatePredictor(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        pf = ParticleFilter(
            paths=[
                '/home/cdondrup/PhD/shaping_hrsi/hmms/Bristol/qtcbc.hmm',
                '/home/cdondrup/PhD/shaping_hrsi/hmms/Bristol/qtcbc_empty.hmm'
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
                "qtcbs": QTCBPathCrossing()
            }
        }
        self.prior = rospy.get_param("~prediction_prior", "passby")
        self.pub = rospy.Publisher("~prediction_array", QTCPredictionArray, queue_size=10)
        rospy.Subscriber(rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array"), QTCArray, self.callback)
        rospy.loginfo("... all done")

    def callback(self, msg):
        interaction_type = self.prior # No classification yet
        start = time.time()
        out = QTCPredictionArray()
        out.header = msg.header
        for q in msg.qtc:
            m = QTCPrediction()
            m.uuid = q.uuid
            try:
                prediction = self.sm[interaction_type][q.qtc_type].predict(
                    json.loads(q.qtc_serialised)[-1],
                    json.loads(q.prob_distance_serialised)[-1]
                )
            except KeyError:
                print q.qtc_type + " not defined"
                return
            print prediction
            m.qtc_serialised = json.dumps(prediction)
            out.qtc.append(m)
        self.pub.publish(out)
        print "elapsed:", time.time() - start


if __name__ == "__main__":
    rospy.init_node("qtc_state_predictor")
    s = StatePredictor(rospy.get_name())
    rospy.spin()