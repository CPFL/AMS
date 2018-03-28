#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


SIM_TAXI_USER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "SimTaxiUser",
    "TRIGGER": {
        "REQUEST": "request",
        "WAIT": "wait",
        "GET_ON": "get_on",
        "GOT_ON": "got_on",
        "MOVE_VEHICLE": "move_vehicle",
        "GET_OUT": "get_out",
        "GOT_OUT": "got_out"
    },
    "STATE": {
        "CALLING": "calling",
        "WAITING": "waiting",
        "GETTING_ON": "getting_on",
        "GOT_ON": "got_on",
        "MOVING": "moving",
        "GETTING_OUT": "getting_out",
        "GOT_OUT": "got_out"
    },
})
