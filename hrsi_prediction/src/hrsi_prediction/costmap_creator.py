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


class CostmapCreator(object):
    def __init__(self, map_pub, origin_pub, width=100, height=100, max_costs=100, min_costs=0, resolution=0.05):
        self._width = width
        self._height = height
        self._max_costs = max_costs
        self._min_costs = min_costs
        self._resolution = resolution
        self._map_pub = map_pub
        self._origin_pub = origin_pub

        self.lock = Lock()

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, width):
        with self.lock:
            self._width = width

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, height):
        with self.lock:
            self._height = height

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

    def _fast_cost(self, angle, qtc_symbol, size=np.pi/2):
        with self.lock: # Making sure no dynamic variable are changed during calculation
            start_loop = time.time()
            cost_array = np.empty((self.width,self.height))
            cost_array.fill(self.max_costs)
            if qtc_symbol == 0:
                return cost_array
            if qtc_symbol == 1:
                angle = angle + np.pi if angle < 0.0 else angle - np.pi

            cp = self._cartesian_product(
                [
                    np.arange(-self.width/2, self.width/2, step=1),
                    np.arange(-self.height/2, self.height/2, step=1)
                ]
            )
            polar = self._cartesian_to_polar(cp[:,0],cp[:,1])
            idx = np.logical_and(
                polar[0] <= min(self.width/2, self.height/2),
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
            ).reshape(self.width,self.height)
            cost_array[idx] = self.min_costs
            print "elapsed:", time.time() - start_loop
            return cost_array.T # Transpose due to shit interpretation of the array by ROS


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

    def publish(self, angle, qtc_symbol, size):
        o = OccupancyGrid()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = 'base_link'
        o.info.resolution = self.resolution
        o.info.height = self._height
        o.info.width = self._width
        o.info.origin = Pose()
        o.info.origin.position.x -= (o.info.width/2) * o.info.resolution # Translate to be centered under robot
        o.info.origin.position.y -= (o.info.height/2)  * o.info.resolution
        p = PoseStamped(header=o.header, pose=o.info.origin)
        self._origin_pub.publish(p)

        o.data = self._fast_cost(angle=angle, qtc_symbol=qtc_symbol, size=size).flatten(order='C')
        self._map_pub.publish(o)

    def _cartesian_to_polar(self, x, y):
        rho = np.sqrt(np.power(x,2) + np.power(y,2))
        phi = np.arctan2(y, x)
        return(rho, phi)

    def _polar_to_cartesian(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x.astype(int), y.astype(int))
