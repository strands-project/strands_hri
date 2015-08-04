#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 16:07:34 2015

@author: cdondrup
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import time
from threading import Lock
import json


class CostmapCreator(object):
    _angle_calculation_table = {
        '?': [lambda x: x],
        -1: [lambda x: x],
        1:  [lambda x: x + np.pi if x < 0.0 else x - np.pi],
        0:
            [
                lambda x: x + np.pi/2 if x + np.pi/2 <= np.pi else -(np.pi - ((np.abs(x) + np.pi/2) - np.pi)),
                lambda x: x - np.pi/2 if x - np.pi/2 >= -np.pi else np.pi - ((np.abs(x) + np.pi/2) - np.pi)
            ]
    }

    _size_lookup_table = {
        '?': 2*np.pi,
        -1: np.pi,
        1:  np.pi,
        0:  0 #np.pi/64
    }

    def __init__(self, map_pub, origin_pub, width=100, height=100, max_costs=100, min_costs=0, resolution=0.05):
#        self._width = width
#        self._height = height
        self._max_costs = max_costs
        self._min_costs = min_costs
        self._resolution = resolution
        self._map_pub = map_pub
        self._origin_pub = origin_pub

        local_planner_name = rospy.get_param("/move_base/base_local_planner").split('/')[1]
        self._max_vel_x_parma_name = "/move_base/" + local_planner_name + "/max_vel_x"
#        self._max_rot_vel_parma_name = "/move_base/" + local_planner_name + "/max_rot_vel"

        self.lock = Lock()

#    @property
#    def width(self):
#        return self._width
#
#    @width.setter
#    def width(self, width):
#        with self.lock:
#            self._width = width
#
#    @property
#    def height(self):
#        return self._height
#
#    @height.setter
#    def height(self, height):
#        with self.lock:
#            self._height = height

    @property
    def max_costs(self):
        return self._max_costs

    @max_costs.setter
    def max_costs(self, max_costs):
        with self.lock:
            self._max_costs = max_costs

    @property
    def min_costs(self):
        return self._min_costs

    @min_costs.setter
    def min_costs(self, min_costs):
        with self.lock:
            self._min_costs = min_costs

    @property
    def resolution(self):
        return self._resolution

    @resolution.setter
    def resolution(self, resolution):
        with self.lock:
            self._resolution = resolution

    def _create_costmap(self, angle=0.0, qtc_symbol=0, size=100):
        cost_array = None
        for f in self._angle_calculation_table[qtc_symbol]:
            cost_array = self._fast_cost(angle=f(angle), qtc_symbol=qtc_symbol, map_size=size, cost_array=cost_array)
#        with open('/home/cdondrup/Desktop/cost_test.csv', 'w') as f:
#            json.dump(cost_array.tolist(), f)
        # Transpose due to shit interpretation of the array by ROS
        return cost_array.T

    def _fast_cost(self, angle=0.0, qtc_symbol=0, map_size=100, cost_array=None):
        start_loop = time.time()
        size = self._size_lookup_table[qtc_symbol]

        if cost_array == None:
            cost_array = np.empty((map_size, map_size))
            cost_array.fill(self.max_costs)

        cp = self._cartesian_product(
            [
                np.arange(-int(np.floor(float(map_size)/2)), int(np.ceil(float(map_size)/2)), step=1),
                np.arange(-int(np.floor(float(map_size)/2)), int(np.ceil(float(map_size)/2)), step=1)
            ]
        )
        polar = self._cartesian_to_polar(cp[:,0],cp[:,1])
        idx = np.logical_and(
            polar[0] <= min(map_size/2, map_size/2), # Not really necessary sinze it is always square
            np.logical_and(
                np.logical_or(
                    polar[1] >= -(size/2) + angle,
                    polar[1] <= -(np.pi - ((np.abs(angle)+(size/2))-np.pi)) # Sign flip
                ),
                np.logical_or(
                    polar[1] <= (size/2) + angle,
                    polar[1] >= np.pi - ((np.abs(angle)+(size/2))-np.pi) # Sign flip
                )
            )
        ).reshape(map_size, map_size)
        cost_array[idx] = self.min_costs
        cost_array[
            int(np.floor(float(map_size)/2))-1:int(np.ceil(float(map_size)/2))+1,
            int(np.floor(float(map_size)/2))-1:int(np.ceil(float(map_size)/2))+1
        ] = self.min_costs # (0,0) should never have costs
        print "elapsed:", time.time() - start_loop
        return cost_array


    def _cartesian_product(self, arrays, out=None):
        """
        Generate a cartesian product of input arrays.

        Parameters
        ----------
        arrays : list of array-like
            1-D arrays to form the cartesian product of.
        out : ndarray
            Array to place the cartesian product in.

        Returns
        -------
        out : ndarray
            2-D array of shape (M, len(arrays)) containing cartesian products
            formed of input arrays.

        Examples
        --------
        >>> cartesian(([1, 2, 3], [4, 5], [6, 7]))
        array([[1, 4, 6],
               [1, 4, 7],
               [1, 5, 6],
               [1, 5, 7],
               [2, 4, 6],
               [2, 4, 7],
               [2, 5, 6],
               [2, 5, 7],
               [3, 4, 6],
               [3, 4, 7],
               [3, 5, 6],
               [3, 5, 7]])

        """

        arrays = [np.asarray(x) for x in arrays]
        dtype = arrays[0].dtype

        n = np.prod([x.size for x in arrays])
        if out is None:
            out = np.zeros([n, len(arrays)], dtype=dtype)

        m = n / arrays[0].size
        out[:,0] = np.repeat(arrays[0], m)
        if arrays[1:]:
            self._cartesian_product(arrays[1:], out=out[0:m,1:])
            for j in xrange(1, arrays[0].size):
                out[j*m:(j+1)*m,1:] = out[0:m,1:]
        return out

    def publish(self, angle, qtc_symbol):
        with self.lock: # Making sure no dynamic variable are changed during calculation
            start = time.time()
            try:
                size = int(rospy.get_param(self._max_vel_x_parma_name)*100*2) # Magic numbers: 100 make int, 2 double the size to have max_vel_x in all directions
            except KeyError as e:
                rospy.logwarn("No such parameter: " + e.message)
                return
#            size = 100
#            print rospy.get_param(self._max_rot_vel_parma_name)
            o = OccupancyGrid()
            o.header.stamp = rospy.Time.now()
            o.header.frame_id = 'base_link'
            o.info.resolution = self.resolution
            o.info.height = size
            o.info.width = size
            o.info.origin = Pose()
            o.info.origin.position.x -= (o.info.width/2) * o.info.resolution # Translate to be centered under robot
            o.info.origin.position.y -= (o.info.height/2)  * o.info.resolution
            p = PoseStamped(header=o.header, pose=o.info.origin)
            self._origin_pub.publish(p)

            o.data = self._create_costmap(angle=angle, qtc_symbol=qtc_symbol, size=size).flatten(order='C')
            self._map_pub.publish(o)
            print "total elapsed:", time.time() - start

    def _cartesian_to_polar(self, x, y):
        rho = np.sqrt(np.power(x,2) + np.power(y,2))
        phi = np.arctan2(y, x)
        return(rho, phi)

    def _polar_to_cartesian(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x.astype(int), y.astype(int))
