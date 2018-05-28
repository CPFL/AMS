#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from ams import get_namedtuple_from_dict


AUTOWARE = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "Autoware",
    "FLOAT_MAX": float_info.max,
    "DEFAULT_UPPER_DISTANCE_FROM_STOPLINE": 50.0,
    "ROS": {
        "DECISION_MAKER_STATES": {
            "MAIN": {
                "START": "Start",
                "DRIVE": "Drive",
                "INITIAL": "Initial",
                "MISSION_COMPLETE": "MissionComplete",
            },
            "ACC": {
                "KEEP": "Keep",
                "STOP": "Stop",
            },
            "STR": {
                "NONE": ""
            },
            "BEHAVIOR": {
                "TRAFFIC_LIGHT_GREEN": "\nTrafficLightGreen",
                "TRAFFIC_LIGHT_RED": "\nTrafficLightRed",
                "WAIT_ORDERS": "\nWaitOrders",
            }
        },
        "STATE_CMD": {
            "MAIN": {
                "START": 1,
                "INIT": 2,
                "DRIVE": 6
            },
            "SUB": {
                "KEEP": 13,
                "STOP": 14
            },
        },
        "TRAFFIC_LIGHT": {
            "RED": 0,
            "GREEN": 1,
            "UNKNOWN": 2
        },
    },
    "TRIGGER": {
        "LAUNCH": "launch",
        "ACTIVATE": "activate",
        "SCHEDULE": "schedule",
        "SYNCHRONIZE_ROUTE": "synchronize_route",
        "MOVE": "move",
        "STOP": "stop",
        "GET_READY": "get_ready"
    },
    "STATE": {
        "LAUNCHED": "launched",
        "STAND_BY": "stand_by",
        "SCHEDULE_UPDATED": "schedule_updated",
        "READY_TO_MOVE": "ready_to_move",
        "MOVE": "move",
        "STOP": "stop",
    },
    "TOPIC": {
        "ROS_NODE_NAME": "ros",
        "CATEGORIES": {
            "BASED_LANE_WAYPOINTS_ARRAY": ["based", "lane_waypoints_array"],
            "STATE_CMD": ["state_cmd"],
            "LIGHT_COLOR": ["light_color"],
            "CURRENT_POSE": ["current_pose"],
            "CLOSEST_WAYPOINT": ["closest_waypoint"],
            "DECISION_MAKER_STATES": ["decisionmaker", "states"],
        }
    },
})
