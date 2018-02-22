#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from ams.structures import get_namedtuple_from_dict


AUTOWARE = get_namedtuple_from_dict("CONST", {
    "FLOAT_MAX": float_info.max,
    "STATE": {
        "STOP": "stop",
        "MOVE": "move"
    },
    "TRAFFIC_LIGHT": {
        "RED": 0,
        "GREEN": 1,
        "UNKNOWN": 2
    },
    "ROSNODE": {
        "AMS_CLOSEST_WAYPOINT_SUBSCRIBER": "ams_closest_waypoint_subscriber",
        "AMS_LANE_ARRAY_PUBLISHER": "ams_lane_array_publisher",
        "AMS_TRAFFIC_LIGHT_PUBLISHER": "ams_traffic_light_publisher",
    },
    "ROSTOPIC": {
        "WAYPOINTS": "/based/lane_waypoints_array",
        "CLOSEST_WAYPOINT": "/closest_waypoint",
        "LIGHT_COLOR": "/light_color_managed"
    },
    "TOPIC": {
        "PUBLISH": "pub_autoware",
        "SUBSCRIBE": "sub_autoware",
        "WAYPOINTS": "/waypoints",
        "CURRENT_POSE": "/current_pose",
        "CLOSEST_WAYPOINT": "/closest_waypoint",
        "TRAFFIC_LIGHT": "/traffic_light"
    },
})
