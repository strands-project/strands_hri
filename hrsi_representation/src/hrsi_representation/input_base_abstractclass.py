#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 11:36:24 2015

@author: cdondrup
"""

import rospy
import numpy as np
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
from qsrlib.qsrlib import QSRlib_Request_Message
from abc import abstractmethod, ABCMeta
try:
    import cPickle as pickle
except:
    import pickle
from collections import OrderedDict


class InputBaseAbstractclass(object):
    """Provides functions for:
        - the transformation of raw data into qsr_lib format
        - converting the data into QTC using qsr_lib

    Will be used as a base class for the training and online data input classes
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        """Creates a new instance of the InputBaseClass"""
        self.qtc_types = OrderedDict([
            ("qtcbs", "qtc_b_simplified"),
            ("qtccs", "qtc_c_simplified"),
            ("qtcbcs", "qtc_bc_simplified"),
            ("qtcbcs_argprobd", "qtc_bc_simplified_arg_prob_distance")
        ])
        self.argprobd = "arg_prob_relations_distance"
        self.template = {
            "agent1": {
                "name": "",
                "x": np.array([]),
                "y": np.array([])
            },
            "agent2": {
                "name": "",
                "x": np.array([]),
                "y": np.array([])
            }
        }
        self.qtc = None

    def _request_qtc(self, qsr, world, parameters, include_missing_data=True, qsrs_for=[]):
        """reads all .qtc files from a given directory and resturns them as numpy arrays"""

        qrmsg = QSRlib_Request_Message(
            which_qsr=qsr,
            input_data=world,
            include_missing_data=include_missing_data,
            qsrs_for=qsrs_for,
            dynamic_args=parameters,
            future=True
        )
        cln = QSRlib_ROS_Client()
        req = cln.make_ros_request_message(qrmsg)
        res = cln.request_qsrs(req)
        out = pickle.loads(res.data)
        rospy.logdebug("Request was made at " + str(out.timestamp_request_made) + " and received at " + str(out.timestamp_request_received) + " and computed at " + str(out.timestamp_qsrs_computed) )
        qtc = np.array([])
        dis = []
        for t in out.qsrs.get_sorted_timestamps():
            for k, v in out.qsrs.trace[t].qsrs.items():
                if len(v.qsr.items()) < 2:
                    continue # Hacky but we only want dist when qtc is there too.
                for l, w in v.qsr.items():
                    if l.startswith("qtc"):
                        q = self._to_np_array(w)
                        if l.startswith(self.qtc_types["qtcbcs"]):
                            q = q if len(q) == 4 else np.append(q, [np.nan, np.nan])
                        qtc = np.array([q]) if not qtc.size else np.append(qtc, [q], axis=0)
                    elif l == "argprobd":
                        dis.append(w)

        return qtc, dis

    def _convert_to_world(self, data_dict):
        world = World_Trace()

        agent1 = data_dict["agent1"]
        agent2 = data_dict["agent2"]

        name1 = agent1["name"]
        name2 = agent2["name"]

        x1 = np.array(agent1["x"], dtype=float)
        y1 = np.array(agent1["y"], dtype=float)

        x2 = np.array(agent2["x"], dtype=float)
        y2 = np.array(agent2["y"], dtype=float)

        ob = []
        for idx, (e_x1, e_y1, e_x2, e_y2) in enumerate(zip(x1,y1,x2,y2)):
            ob.append(Object_State(
                name=name1,
                timestamp=idx,
                x=e_x1,
                y=e_y1
            ))
            ob.append(Object_State(
                name=name2,
                timestamp=idx,
                x=e_x2,
                y=e_y2
            ))

        world.add_object_state_series(ob)
        return world

    @abstractmethod
    def generate_data_from_input(self, *args, **kwargs):
        """Input data into the conversion process"""
        pass

    def convert(self, data, qtc_type, parameters):
        """Convert data inserted via put() into QTC

        :param qtc_type: qtcb|qtcc|qtcbc
        """
        data = [data] if not isinstance(data, list) else data
        ret = []
        for elem in np.array(data):
            world = self._convert_to_world(data_dict=elem)

            try:
                qsr = [self.qtc_types[qtc_type], self.argprobd]
            except KeyError:
                rospy.logfatal("Unknown QTC type: %s" % qtc_type)
                return
            ret.append(self._request_qtc(qsr=qsr, world=world, parameters=parameters, qsrs_for=[(elem["agent1"]["name"],elem["agent2"]["name"])]))
        return ret

    def _to_np_array(self, string):
        string = string.values()[0] if isinstance(string, dict) else string
        return np.fromstring(string.replace('-','-1').replace('+','+1'), dtype=int, sep=',')
