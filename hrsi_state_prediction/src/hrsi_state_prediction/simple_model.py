# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 17:09:06 2015

@author: cdondrup
"""

class SimpleModel(object):
    _state_chain = [
        lambda x,y: ['?',x[1]] if x[1] ==  0 and y not in ("per", "int") else None,
        lambda x,y: [ -1,x[1]] if x[1] == -1 and y not in ("soc", "per", "int") else None,
        lambda x,y: [  0,x[1]] if x[1] == -1 and y not in ("pub",) else None,
        lambda x,y: [  0,x[1]] if x[1] ==  0 and y not in ("pub", "soc") else None,
        lambda x,y: [  0,x[1]] if x[1] ==  1 and y not in ("pub", "soc") else None,
        lambda x,y: [  1,x[1]] if x[1] ==  1 and y not in ("per", "int") else None
    ]

    def __init__(self):
        self.previous_state = ['?',0]
        pass

    def predict(self, current_state, distance):
        print current_state, distance
        for f in self._state_chain:
            s = f(current_state, distance)
            if s:
                self.previous_state = s
                break
        return self.previous_state
