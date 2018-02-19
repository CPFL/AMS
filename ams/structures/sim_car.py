#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from ams.structures import get_namedtuple_from_dict


SIM_CAR = get_namedtuple_from_dict("CONST", {
    "LOWER_INTER_VEHICLE_DISTANCE": 3.0,
    "LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE": 1.0,
    "FLOAT_MAX": float_info.max,
    "ACCELERATION_MAX": 0.3  # [m/s^2]
})
