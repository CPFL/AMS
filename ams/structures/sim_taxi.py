#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


SIM_TAXI = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "SimTaxi",
    "TRIGGER": {
        "STAND_BY": "stand_by",
        "MOVE": "move",
        "STOP": "stop",
    },
    "STATE": {
        "STAND_BY": "stand_by",
        "MOVE_FOR_PICKING_UP": "move_for_picking_up",
        "STOP_FOR_PICKING_UP": "stop_for_picking_up",
        "MOVE_FOR_DISCHARGING": "move_for_discharging",
        "STOP_FOR_DISCHARGING": "stop_for_discharging"
    },
})
