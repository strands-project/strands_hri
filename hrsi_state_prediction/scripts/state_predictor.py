#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 16:57:54 2015

@author: cdondrup
"""

import rospy
from hrsi_representation.msg import QTCArray
from hrsi_state_prediction.simple_model import SimpleModel
from hrsi_state_prediction.msg import QTCPrediction, QTCPredictionArray
import json
import time


class StatePredictor(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.sm = SimpleModel()
        self.pub = rospy.Publisher("~prediction_array", QTCPredictionArray, queue_size=10)
        rospy.Subscriber(rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array"), QTCArray, self.callback)
        rospy.loginfo("... all done")

    def callback(self, msg):
        start = time.time()
        out = QTCPredictionArray()
        out.header = msg.header
        for q in msg.qtc:
            m = QTCPrediction()
            m.uuid = q.uuid
            prediction = self.sm.predict(
                json.loads(q.qtc_serialised)[-1],
                json.loads(q.prob_distance_serialised)[-1]
            )
            print prediction
            m.qtc_serialised = json.dumps(prediction)
            out.qtc.append(m)
        self.pub.publish(out)
        print "elapsed:", time.time() - start


if __name__ == "__main__":
    rospy.init_node("qtc_state_predictor")
    s = StatePredictor(rospy.get_name())
    rospy.spin()