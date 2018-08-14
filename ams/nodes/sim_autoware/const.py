#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


topic = {
    "CATEGORIES": {
        "LANE_WAYPOINTS_ARRAY": ["based", "lane_waypoints_array"],
        "STATE_CMD": ["state_cmd"],
        "LIGHT_COLOR": ["light_color"],
        "CURRENT_POSE": ["current_pose"],
        "CLOSEST_WAYPOINT": ["closest_waypoint"],
        "DECISION_MAKER_STATE": ["decision_maker", "state"]
    }
}

const = {
    "NODE_NAME": "sim_autoware",
    "ROLE_NAME": "autoware",
    "TOPIC": topic,
    "DECISION_MAKER_STATE": {
        "WAIT_MISSION_ORDER": "WaitMissionOrder\n",
        "WAIT_ORDER": "WaitOrder\n",
        "MISSION_CHECK": "MissionCheck\n",
        "DRIVE_READY": "DriveReady\n",
        "DRIVE": "Drive\n"
    },
    "STATE_CMD": {
        "ENGAGE": "engage"
    },
    "TRAFFIC_LIGHT": {
        "RED": 0,
        "GREEN": 1,
        "UNKNOWN": 2
    }
}

CONST = get_namedtuple_from_dict("CONST", const)
