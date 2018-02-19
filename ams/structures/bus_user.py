#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


BUS_USER = get_namedtuple_from_dict("CONST", {
    "TARGET_GROUP": {
        "START_BUS_STOP": "startBusStop",
        "GOAL_BUS_STOP": "goalBusStop"
    },
    "STATE": {
        "ARRIVED_AT_BUS_STOP": "arrivedAtBusStop",
        "WAITING": "waiting",
        "GETTING_ON": "gettingOn",
        "GOT_ON": "gotOn",
        "MOVING": "moving",
        "READY_TO_GET_OUT": "readyToGetOut",
        "GETTING_OUT": "gettingOut",
        "GOT_OUT": "gotOut"
    },
    "ACTION": {
        "WAIT": "wait",
        "GET_ON": "getOn",
        "REQUEST_STOP": "requestStop",
        "GET_OUT": "getOut"
    },
    "EVENT": {
        "MOVE_VEHICLE": "moveVehicle"
    }
})
