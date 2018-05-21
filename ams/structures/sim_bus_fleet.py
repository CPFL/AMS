#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


SIM_BUS_FLEET = get_namedtuple_from_dict("CONST", {
    "DISPATCHABLE_GEOHASH_DIGIT": 6,
    "TIMEOUT": 30.0,
    "TARGET_GROUP": {
        "BUS_SCHEDULES": "bus_schedules"
    },
    "TRIGGER": {
        "SEND_CIRCULAR_ROUTE_SCHEDULES": "send_circular_route_schedules",
        "SEND_THROUGH_SCHEDULES": "send_through_schedules",
        "SEND_VIA_SCHEDULES": "send_via_schedules"
    },
    "STATE": {
        "WAITING_FOR_BUS_STAND_BY": "waiting_for_bus_stand_by",
        "WAITING_FOR_SCHEDULES_REQUEST": "waiting_for_schedules_request",
    }
})
