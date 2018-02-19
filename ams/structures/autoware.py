#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from ams.structures import get_namedtuple_from_dict


AUTOWARE = get_namedtuple_from_dict("CONST", {
    "FLOAT_MAX": float_info.max,
    "TRAFFIC_LIGHT": {
        "RED": 0,
        "GREEN": 1,
        "UNKNOWN": 2
    },
    "ROSTOPIC": {
        "WAYPOINTS": "/based/lane_waypoints_array",
        "TRAFFIC_LIGHT": "/light_color_managed",
        "CLOSEST_WAYPOINT": "/closest_waypoint"
    },
    "TOPIC": {
        "PUBLISH": "pub_autoware",
        "SUBSCRIBE": "sub_autoware"
    },
})
