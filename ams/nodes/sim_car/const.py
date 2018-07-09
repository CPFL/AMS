#!/usr/bin/env python
# coding: utf-8

from sys import float_info

from ams import get_namedtuple_from_dict
from ams.nodes.vehicle.const import const as vehicle_const
from ams.nodes.vehicle.const import mission_event as vehicle_mission_event


topic = {
    "CATEGORIES": {
        "LOCATION": ["status", "location"]
    }
}
topic["CATEGORIES"].update(vehicle_const["TOPIC"]["CATEGORIES"])

mission_event = {
    "MOVE": "move",
    "STOP": "stop"
}
mission_event.update(vehicle_mission_event)

event = {}
event.update(vehicle_const["EVENT"])
event.update(mission_event)

state = {
    "MOVE": "move",
    "STOP": "stop"
}
state.update(vehicle_const["STATE"])

const = {
    "NODE_NAME": "sim_car",
    "TOPIC": topic,
    "EVENT": event,
    "STATE": state,
    "MISSION_EVENTS": mission_event.values(),
    "DEFAULT_VELOCITY": 3.0,
    "LOWER_INTER_VEHICLE_DISTANCE": 3.0,
    "LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE": 1.0,
    "FLOAT_MAX": float_info.max,
    "ACCELERATION_MAX": 0.3  # [m/s^2]
}

CONST = get_namedtuple_from_dict("CONST", const)
