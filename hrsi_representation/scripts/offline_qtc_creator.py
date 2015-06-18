#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 30 15:28:58 2015

@author: cdondrup
"""

import rospy
import argparse
from hrsi_representation.file_input import FileInput


class OfflineQTCCreator(object):
    """trains hmm from raw data files"""

    def __init__(self, name, args):
        """Creates a new instance of the class
        """
        rospy.loginfo("Starting %s", name)
        self.parameters = {
            "quantisation_factor": rospy.get_param("~quantisation_factor", 0.01),
            "validate":            rospy.get_param("~validate", True),
            "no_collapse":         rospy.get_param("~no_collapse", False),
            "distance_threshold":  rospy.get_param("~distance_threshold", 1.2)
        }

        self.input = rospy.get_param("~input")
        self.output = rospy.get_param("~output")
        self.qsr = rospy.get_param("~qsr", "qtcc")

        if not self.input:
            rospy.logfatal("No input path provided. Cannot load files.")

        self.file_input = FileInput()

    def create(self):
        rospy.loginfo("Reading file '%s':" % self.i)
        qtc = self.file_input.convert(
            data=self.file_input.generate_data_from_input(path=self.i),
            qtc_type=self.qsr,
            quantisation_factor=self.q,
            validate=self.v,
            no_collapse=self.n,
            distance_threshold=self.d
        )
        return qtc


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("qsr", help="choose qsr: qtcc|qtcc|qtcbc", type=str)
    parser.add_argument("-i", "--input", help="path under which to read csv files", type=str)
    parser.add_argument("--output", help="path in which to store the resulting qtc files", type=str)
    parser.add_argument("--quantisation_factor", help="quantisation factor for 0-states.", type=float)
    parser.add_argument("--no_collapse", help="does not collapse similar adjacent states.", action="store_true")
    parser.add_argument("--distance_threshold", help="distance threshold for qtcb <-> qtcc transition. Only QTCBC", type=float)
    args = parser.parse_args()

    rospy.init_node("offline_qtc_creator")
    t = OfflineQTCCreator(rospy.get_name(), args)
    qtc = t.train()
    print qtc
    #rospy.spin()