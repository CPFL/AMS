#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


BUS_FLEET = get_namedtuple_from_dict("CONST", {
    "DISPATCHABLE_GEOHASH_DIGIT": 6,
    "TIMEOUT": 30.0,
    "UNASSIGNED_BUS_SCHEDULES_KEY": "unassigned",
    "UNASSIGNED_BUS_SCHEDULES_VALUE": "bus_schedules",
})
