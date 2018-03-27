#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from ams.structures import get_namedtuple_from_dict


AUTOWARE_VOICE_RECOGNITION = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "AutowareVoiceRecognition",
    "FLOAT_MAX": float_info.max,
    "STATE": {
        "STOP": "stop",
        "MOVE": "move",
        "ACCELERATE": "accelerate",
        "DECELERATE": "decelerate"
    },
    "TOPIC": {
        "CATEGORIES": {
            "STATUS": ["status"],
            "LANE_ARRAY": ["lane_array"],
            "STATE_COMMAND": ["state_command"],
            "LIGHT_COLOR": ["light_color"],
            "INITIAL_POSE": ["initial_pose"],
            "VOICE": ["voice"],
        }
    },
    "VOICE_TYPE": {
        "CAR_CONTROL": "car_control",
        "DESTINATION_SET": "destination_set",
    }
})
