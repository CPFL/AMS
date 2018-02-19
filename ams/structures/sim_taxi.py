#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


SIM_TAXI = get_namedtuple_from_dict("CONST", {
    "STATE": {
        "STANDBY": "standBy",
        "MOVE_TO_USER": "moveToUser",
        "STOP_FOR_PICKING_UP": "pickingUp",
        "MOVE_TO_USER_DESTINATION": "moveToUserDestination",
        "STOP_FOR_DISCHARGING": "discharging",
        "MOVE_TO_STANDBY": "moveToDeploy"
    },
})
