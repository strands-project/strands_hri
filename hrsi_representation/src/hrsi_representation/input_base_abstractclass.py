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


class InputBaseAbstractclass(object):
    """Provides functions for:
        - the transformation of raw data into qsr_lib format
        - converting the data into QTC using qsr_lig

    Will be used as a base class for the training and online data input classes
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        """Creates a new instance of the InputBaseClass"""
        self.qtc_types = {
            "qtcb": "qtc_b_simplified",
            "qtcc": "qtc_c_simplified",
            "qtcbc": "qtc_bc_simplified"
        }
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

    def _request_qtc(self, qsr, world, include_missing_data=True, qsrs_for=[]):
        """reads all .qtc files from a given directory and resturns them as numpy arrays"""

        qrmsg = QSRlib_Request_Message(which_qsr=qsr, input_data=world, include_missing_data=include_missing_data, qsrs_for=qsrs_for)
        cln = QSRlib_ROS_Client()
        req = cln.make_ros_request_message(qrmsg)
        res = cln.request_qsrs(req)
        out = pickle.loads(res.data)
        rospy.logdebug("Request was made at " + str(out.timestamp_request_made) + " and received at " + str(out.timestamp_request_received) + " and computed at " + str(out.timestamp_qsrs_computed) )
        ret = np.array([])
        for t in out.qsrs.get_sorted_timestamps():
            foo = str(t) + ": "
            for k, v in zip(out.qsrs.trace[t].qsrs.keys(), out.qsrs.trace[t].qsrs.values()):
                foo += str(k) + ":" + str(v.qsr) + "; "
                if qsr == self.qtc_types["qtcbc"]:
                    q = v.qsr if len(v.qsr) == 4 else np.append(v.qsr, [np.nan, np.nan])
                    ret = np.array([q]) if not ret.size else np.append(ret, [q], axis=0)
                else:
                    ret = np.array([v.qsr]) if not ret.size else np.append(ret, [v.qsr], axis=0)
            rospy.logdebug(foo)

        return ret

    def _convert_to_world(self, data_dict, quantisation_factor=0, validate=True, no_collapse=False, distance_threshold=np.Inf):
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
                y=e_y1,
                quantisation_factor=quantisation_factor,
                validate=validate,
                no_collapse=no_collapse,
                distance_threshold=distance_threshold
            ))
            ob.append(Object_State(
                name=name2,
                timestamp=idx,
                x=e_x2,
                y=e_y2,
                quantisation_factor=quantisation_factor,
                validate=validate,
                no_collapse=no_collapse,
                distance_threshold=distance_threshold
            ))

        world.add_object_state_series(ob)
        return world

    @abstractmethod
    def generate_data_from_input(self, *args, **kwargs):
        """Input data into the conversion process"""
        pass

    def convert(self, data, qtc_type, quantisation_factor=0, validate=True, no_collapse=False, distance_threshold=np.Inf):
        """Convert data inserted via put() into QTC

        :param qtc_type: qtcb|qtcc|qtcbc
        """
        data = [data] if not isinstance(data, list) else data
        ret = []
        for elem in np.array(data):
            world = self._convert_to_world(
                data_dict=elem,
                quantisation_factor=quantisation_factor,
                validate=validate,
                no_collapse=no_collapse,
                distance_threshold=distance_threshold
            )

            try:
                qsr = self.qtc_types[qtc_type]
            except KeyError:
                rospy.logfatal("Unknown QTC type: %s" % qtc_type)
                return
            ret.append(self._request_qtc(qsr=qsr, world=world, qsrs_for=[(elem["agent1"]["name"],elem["agent2"]["name"])]))
        return ret
