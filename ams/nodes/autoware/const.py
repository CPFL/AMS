#!/usr/bin/env python
# coding: utf-8

from sys import float_info

from ams import get_namedtuple_from_dict
from ams.nodes.vehicle.const import const as vehicle_const
from ams.nodes.vehicle.const import mission_event as vehicle_mission_event


topic = {
    "CATEGORIES": {
        "BASED_LANE_WAYPOINTS_ARRAY": ["based", "lane_waypoints_array"],
        "STATE_CMD": ["state_cmd"],
        "LIGHT_COLOR": ["light_color"],
        "CURRENT_POSE": ["current_pose"],
        "CLOSEST_WAYPOINT": ["closest_waypoint"],
        "DECISION_MAKER_STATE": ["decision_maker", "state"],
    }
}
topic["CATEGORIES"].update(vehicle_const["TOPIC"]["CATEGORIES"])

mission_event = {
    "SEND_LANE_WAYPOINT_ARRAY": "send_lane_waypoint_array",
    "SEND_ENGAGE": "send_engage",
}
mission_event.update(vehicle_mission_event)

event = {}
event.update(vehicle_const["EVENT"])
event.update(mission_event)

state = {
    "WAITING_FOR_AUTOWARE_STATE_WAIT_ORDER": "waiting_for_decision_maker_state_wait_order",
    "WAITING_FOR_AUTOWARE_STATE_DRIVE_READY": "waiting_for_decision_maker_state_drive_ready",
}
state.update(vehicle_const["STATE"])

const = {
    "NODE_NAME": "autoware",
    "ROLE_NAME": "vehicle",
    "TOPIC": topic,
    "EVENT": event,
    "STATE": state,
    "ROS": {
        "NODE_NAME": "ros",
        "ROLE_NAME": "ros",
        "DECISION_MAKER_STATE": {
            "WAIT_MISSION_ORDER": "WaitMissionOrder\n",
            "WAIT_ORDER": "WaitOrder\n",
            "DRIVE_READY": "DriveReady\n"
        },
        "STATE_CMD": {
            "ENGAGE": "engage"
        },
        "TRAFFIC_LIGHT": {
            "RED": 0,
            "GREEN": 1,
            "UNKNOWN": 2
        },
    },
    "MISSION_EVENTS": mission_event.values(),
    "FLOAT_MAX": float_info.max,
    "DEFAULT_UPPER_DISTANCE_FROM_STOPLINE": 50.0
}

CONST = get_namedtuple_from_dict("CONST", const)
