#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


topic = {
    "CATEGORIES": {
        "CONFIG": ["config"],
        "STATUS": ["status"],
        "SCHEDULES": ["schedules"],
        "TRANSPORTATION_STATUS": ["transportation", "status"]
    }
}

event = {
    "START": "start",
    "ACTIVATE": "activate",
    "DEACTIVATE": "deactivate",
    "END": "end"
}

state = {
    "START_PROCESSING": "start_processing",
    "ACTIVE": "active",
    "INACTIVE": "inactive",
    "END_PROCESSING": "end_processing"
}

transportation_state = {
    "START_PROCESSING": "start_processing",
    "HANDSHAKE": "handshake",
    "ACTIVE": "active",
    "WAITING_FOR_VEHICLE_STATE_LOCATED": "waiting_for_vehicle_state_located",
    "WAITING_FOR_VEHICLE_STATE_END_PROCESSING": "waiting_for_vehicle_state_end_processing",
    "END_PROCESSING": "end_processing"
}

const = {
    "NODE_NAME": "dispatcher",
    "ROLE_NAME": "dispatcher",
    "TOPIC": topic,
    "EVENT": event,
    "STATE": state,
    "TRANSPORTATION": {
        "STATE": transportation_state
    }
}

CONST = get_namedtuple_from_dict("CONST", const)
