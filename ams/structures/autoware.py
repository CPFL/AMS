#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from ams.structures import get_namedtuple_from_dict


AUTOWARE = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "Autoware",
    "FLOAT_MAX": float_info.max,
    "STATE": {
        "STOP": "stop",
        "MOVE": "move"
    },
    "TOPIC": {
        "CATEGORIES": {
            "STATUS": ["status"],
            "LANE_ARRAY": ["lane_array"],
            "STATE_COMMAND": ["state_command"],
            "LIGHT_COLOR": ["light_color"],
        }
    },
})
