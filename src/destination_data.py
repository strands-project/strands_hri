#! /usr/bin/env python

class Destination_Data(object):
    def __init__(self, name=None, description=None, kind=None, goto=None, available=None):
        self.id = "_".join(name.split())
        self.name = name.replace("_", " ")
        self.description = description
        self.kind = kind
        self.goto = goto.replace("_", " ") # give self.id or id to where to drop off (e.g. lift id)
        self.available = available

    def get_metadata_from_map(self):
        pass