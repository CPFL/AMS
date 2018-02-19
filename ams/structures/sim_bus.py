#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


SIM_BUS = get_namedtuple_from_dict("CONST", {
    "STATE": {
        "MOVE": "move",
        "STOP": "stop",
        "VIA": "requestViaSchedules",
        "THROUGH": "requestThroughSchedules"
    },
    "SCHEDULE": {
        "STAND_BY": "standBy",
        "MOVE_TO_CIRCULAR_ROUTE": "moveToCircularRoute",
        "MOVE_TO_SELECT_POINT": "moveToSelectPoint",
        "MOVE_TO_BRANCH_POINT": "moveToBranchPoint",
        "MOVE_TO_BUS_STOP": "moveToBusStop",
        "MOVE_TO_JUNCTION": "moveToJunction",
        "MOVE_TO_PARKING": "moveToParking",
        "STOP_TO_TAKE_UP": "stopToTakeUp",
        "STOP_TO_DISCHARGE": "stopToDischarge",
        "STOP_TO_DISCHARGE_AND_TAKE_UP": "stopToDischargeAndTakeUp",
        "STOP_TO_PARKING": "stopToParking"
    }
})
