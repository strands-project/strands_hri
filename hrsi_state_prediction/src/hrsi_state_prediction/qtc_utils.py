# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 15:47:20 2015

@author: cdondrup
"""

import numpy as np
import rospy


def create_states(qtc_type):
    try:
        if qtc_type == "":
            raise AttributeError()
        elif qtc_type == 'qtcbs':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2]
        elif qtc_type is 'qtccs':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
        elif qtc_type.startswith('qtcbcs'):
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2, np.NaN, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
    except AttributeError:
        rospy.logfatal("QTC type: %s not found" % qtc_type)

def to_symbol(qtc_data):
    for idx, element in enumerate(qtc_data):
        element = np.array(element)
        d = element.shape[0]
        mult = 3**np.arange(d-1, -1, -1)
        yield ((element + 1)*mult).sum() + 1