#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 16:57:54 2015

@author: cdondrup
"""

import rospy
from hrsi_representation.msg import QTCArray
from hrsi_state_prediction.simple_model import QTCBPassBy, QTCCPassBy, QTCBCPassBy, QTCBPathCrossing, QTCBCPathCrossing, QTCCPathCrossing
from hrsi_state_prediction.particle_filter import ParticleFilterPredictor
from hrsi_state_prediction.state_mapping import StateMapping
from hrsi_state_prediction.msg import QTCPrediction, QTCPredictionArray
from dynamic_reconfigure.server import Server as DynServer
from hrsi_state_prediction.cfg import StatePredictorConfig
import json
from hrsi_state_prediction.srv import LoadModel, LoadModelResponse
import numpy as np
import hrsi_state_prediction.qtc_utils as qu


class StatePredictor(object):
    __interaction_types = ["passby", "pathcrossing"]

    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.dyn_srv = DynServer(StatePredictorConfig, self.dyn_callback)
        self.pf = ParticleFilterPredictor(
            path=rospy.get_param("~model_dir")
        )
#        self.sm = {
#            "passby": {
#                "qtcbs": QTCBPassBy(),
#                "qtccs": QTCCPassBy(),
#                "qtcbcs": QTCBCPassBy(),
#                "qtcbcs_argprobd": QTCBCPassBy()
#            },
#            "pathcrossing": {
#                "qtcbs": QTCBPathCrossing(),
#                "qtccs": QTCCPathCrossing(),
#                "qtcbcs": QTCBCPathCrossing()
#            }
#        }
        self.mapping = StateMapping()
        self.pub = rospy.Publisher("~prediction_array", QTCPredictionArray, queue_size=10)
        rospy.Service("~load_model", LoadModel, self.srv_cb)
        rospy.Subscriber(rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array"), QTCArray, self.callback, queue_size=1)
        rospy.loginfo("... all done")

    def srv_cb(self, req):
        self.mapping.load_model(req.filename)
        return LoadModelResponse()

    def dyn_callback(self, config, level):
        self.prior = self.__interaction_types[config["type"]]
        rospy.loginfo("Interaction type set to %s" % self.prior)
        return config

    def callback(self, msg):
#        interaction_type = self.prior # No classification yet
#        start = time.time()
        out = QTCPredictionArray()
        out.header = msg.header
        for q in msg.qtc:
            m = QTCPrediction()
            m.uuid = q.uuid
            try:
                qtc_robot = json.loads(q.qtc_robot_human)
                qtc_goal = json.loads(q.qtc_goal_human)
                dist = json.loads(q.prob_distance_robot_human)[-1]
#                prediction = self.sm[interaction_type][q.qtc_type].predict(
#                    qtc_robot[-1],
#                    dist
#                )
#                prediction = self.mapping.predict(qtc_robot=qtc_robot, qtc_goal=qtc_goal)
#                prediction = self.pf.predict(m.uuid, qtc, dist)
                qtc_robot = np.array(qtc_robot[-1]); qtc_goal = np.array(qtc_goal[-1])
                qtc_robot[np.isnan(qtc_robot)] = qu.NO_STATE
                qtc_goal[np.isnan(qtc_goal)] = qu.NO_STATE
                qtc = np.append(qtc_goal[[1,3]], qtc_robot[[1,3]])
                prediction = self.pf.predict(m.uuid, [qtc], dist)
            except KeyError:
                print q.qtc_type + " not defined"
                return
            print "~~~~~~~~~~", prediction
            if prediction == None:
                return
            m.qtc_serialised = json.dumps(prediction)
            out.qtc.append(m)
        self.pub.publish(out)
#        print "elapsed:", time.time() - start


if __name__ == "__main__":
    rospy.init_node("qtc_state_predictor")
    s = StatePredictor(rospy.get_name())
    rospy.spin()