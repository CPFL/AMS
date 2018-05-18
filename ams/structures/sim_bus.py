#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


SIM_BUS = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "SimBus",
    "EVENT": {
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
    },
    "TRIGGER": {
        "STAND_BY": "stand_by",
        "MOVE": "move",
        "MOVE_AND_REQUEST_SCHEDULES": "move_and_request_schedules",
        "REQUEST_SCHEDULES": "request_schedules",
        "STOP": "stop",
    },
    "STATE": {
        "STAND_BY": "stand_by",
        "MOVE_TO_CIRCULAR_ROUTE": "move_to_circular_route",
        "MOVE_TO_SELECT_POINT": "move_to_select_point",
        "REQUEST_THROUGH_SCHEDULES": "request_through_schedules",
        "REQUEST_VIA_SCHEDULES": "request_via_schedules",
        "MOVE_TO_BRANCH_POINT": "move_to_branch_point",
        "MOVE_TO_BUS_STOP": "move_to_bus_stop",
        "MOVE_TO_JUNCTION": "move_to_junction",
        "MOVE_TO_PARKING": "move_to_parking",
        "STOP_FOR_DISCHARGING_AND_TAKING_UP": "stop_for_discharging_and_taking_up",
        "STOP_FOR_PARKING": "stop_for_parking"
    }
})
