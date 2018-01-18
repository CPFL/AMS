#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Pose

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


class Waypoint(get_base_class(template, schema)):
    pass
