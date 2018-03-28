#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from ams.structures import get_namedtuple_from_dict


SIM_CAR = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "SimCar",
    "TOPIC": {
        "CATEGORIES": {
            "LOCATION": ["status", "location"],
            "STATUS": ["status"]
        }
    },
    "TRIGGER": {
        "MOVE": "move",
        "STOP": "stop"
    },
    "STATE": {
        "MOVE": "move",
        "STOP": "stop"
    },
    "LOWER_INTER_VEHICLE_DISTANCE": 3.0,
    "LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE": 1.0,
    "FLOAT_MAX": float_info.max,
    "ACCELERATION_MAX": 0.3  # [m/s^2]
})
