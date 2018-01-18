#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator
from ams.structures import Pose

template = {
    "geohash": "123456789012345",
    "pose": Pose.get_template(),
    "speed_limit": 5.5
}

'''
waypoint_id = "[id defined by autoware maps]"
waypoints = {
    waypoint_id: waypoint,
}
'''

schema = {
    "geohash": {
        "type": "string",
        "required": True,
        "nullable": False,
        "minlength": 15,
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "nullable": False,
    },
    "speed_limit": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Waypoint(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)
