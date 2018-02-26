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
    "STATE_MATCHINE": {
        "STATE": {
            "MAIN": {
                "START": 1
            },
            "SUB": {
                "KEEP": 13,
                "STOP": 14
            }
        }
    },
    "TRAFFIC_LIGHT": {
        "RED": 0,
        "GREEN": 1,
        "UNKNOWN": 2
    },
    "ROSNODE": {
        "AMS_CURRENT_POSE_SUBSCRIBER": "ams_current_pose_subscriber",
        "AMS_CLOSEST_WAYPOINT_SUBSCRIBER": "ams_closest_waypoint_subscriber",
        "AMS_LANE_ARRAY_PUBLISHER": "ams_lane_array_publisher",
        "AMS_STATE_COMMAND_PUBLISHER": "ams_state_command_publisher",
        "AMS_TRAFFIC_LIGHT_PUBLISHER": "ams_traffic_light_publisher",
    },
    "ROSTOPIC": {
        "CURRENT_POSE": "/current_pose",
        "WAYPOINTS": "/based/lane_waypoints_array",
        "STATE_COMMAND": "/state_cmd",
        "CLOSEST_WAYPOINT": "/closest_waypoint",
        "LIGHT_COLOR": "/light_color"
    },
    "TOPIC": {
        "PUBLISH": "pub_autoware",
        "SUBSCRIBE": "sub_autoware",
        "WAYPOINTS": "/waypoints",
        "STATE_COMMAND": "/state_command",
        "CURRENT_POSE": "/current_pose",
        "CLOSEST_WAYPOINT": "/closest_waypoint",
        "TRAFFIC_LIGHT": "/traffic_light"
    },
})
