#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


SIM_TAXI_USER = get_namedtuple_from_dict("CONST", {
    "STATE": {
        "CALLING": "calling",
        "WAITING": "waiting",
        "GETTING_ON": "gettingOn",
        "GOT_ON": "gotOn",
        "MOVING": "moving",
        "GETTING_OUT": "gettingOut",
        "GOT_OUT": "gotOut"
    },
    "ACTION": {
        "WAIT": "wait",
        "GET_ON": "getOn",
        "GET_OUT": "getOut"
    },
    "EVENT": {
        "MOVE_VEHICLE": "moveVehicle"
    }
})
