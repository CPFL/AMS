#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


SIM_BUS_USER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "SimBusUser",
    "TARGET_GROUP": {
        "START_BUS_STOP": "startBusStop",
        "GOAL_BUS_STOP": "goalBusStop"
    },
    "TRIGGER": {
        "WAIT": "wait",
        "GET_ON": "get_on",
        "GOT_ON": "got_on",
        "MOVE_VEHICLE": "move_vehicle",
        "REQUEST_STOP": "request_stop",
        "GET_OUT": "get_out",
        "GOT_OUT": "got_out"
    },
    "STATE": {
        "ARRIVED_AT_BUS_STOP": "arrived_at_bus_stop",
        "WAITING": "waiting",
        "GETTING_ON": "getting_on",
        "GOT_ON": "got_on",
        "MOVING": "moving",
        "READY_TO_GET_OUT": "ready_to_get_out",
        "GETTING_OUT": "getting_out",
        "GOT_OUT": "got_out"
    }
})
