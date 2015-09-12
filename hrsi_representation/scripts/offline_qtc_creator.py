#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 30 15:28:58 2015

@author: cdondrup
"""

import rospy
from hrsi_representation.file_input import FileInput
import hrsi_representation.output as output
from hrsi_representation.msg import QTCArray
import json


class OfflineQTCCreator(object):
    """trains hmm from raw data files"""

    def __init__(self, name):
        """Creates a new instance of the class
        """
        rospy.loginfo("Starting %s", name)

        self.input = rospy.get_param("~input", "")
        self.outpath = rospy.get_param("~outpath", "")
        self.qsr = rospy.get_param("~qsr", "qtccs")

        self.parameters = {"qtcs": {
            "quantisation_factor": rospy.get_param("~quantisation_factor", 0.01),
            "validate":            rospy.get_param("~validate", True),
            "no_collapse":         rospy.get_param("~no_collapse", False),
            "distance_threshold":  rospy.get_param("~distance_threshold", 1.2)
        },
            "for_all_qsrs": {
                "qsrs_for": [("robot", "human")]
        }}

        self.pub = rospy.Publisher("~qtc_array", QTCArray, queue_size=10)

        self.file_input = FileInput()

        if self.input == "":
            rospy.logfatal("No input path provided. Cannot load files.")
            return

    def create(self):
        rospy.loginfo("Reading from dir: '%s'" % self.input)
        data, files = self.file_input.generate_data_from_input(path=self.input)
        qtc = self.file_input.convert(
            data=data,
            qtc_type=self.qsr,
            parameters=self.parameters,
            argprobd=False
        )
        return qtc, files, data

    def publish(self, qtc, data, files):
        out = output.create_qtc_array_msg(
            stamp=rospy.Time.now()
        )
        for q, d, f in zip(qtc, data, files):
            m = output.create_qtc_msg(
                collapsed=not self.parameters["qtcs"]["no_collapse"],
                qtc_type=self.qsr,
                k=d["agent1"]["name"],
                l=d["agent2"]["name"],
                quantisation_factor=self.parameters["qtcs"]["quantisation_factor"],
                distance_threshold=self.parameters["qtcs"]["distance_threshold"],
                smoothing_rate=0.0,
                validated=self.parameters["qtcs"]["validate"],
                uuid=f,
                qtc_serialised=json.dumps(q[0].tolist()),
                prob_distance_serialised=[]
            )
            out.qtc.append(m)

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node("offline_qtc_creator")
    t = OfflineQTCCreator(rospy.get_name())
    qtc, files, data = t.create()
    t.publish(qtc, data, files)
    if t.outpath != "":
        output.write_files(qtc,files,t.outpath)
#    for elem, f in zip(qtc, files):
#        print f, elem
    #rospy.spin()


