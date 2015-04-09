#! /usr/bin/env python

class Destination_Data(object):
    def __init__(self, name=None, description=None, kind=None, goto=None, available=True, at_node=None):
        foo = name.replace("_", " ")
        foo = foo.replace("-", " ")
        self.id = "_".join(foo.split())
        self.name = name
        self.description = description
        self.kind = kind
        self.goto = goto
        self.available = available
        self.at_node = at_node

    def get_metadata_from_map(self):
        pass
