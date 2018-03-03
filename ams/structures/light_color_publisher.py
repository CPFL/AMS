#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


LIGHT_COLOR_PUBLISHER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "LightColorPublisher",
    "TRAFFIC_LIGHT": {
        "RED": 0,
        "GREEN": 1,
        "UNKNOWN": 2
    },
    "ROSNODE": "ams_traffic_light_publisher",
    "ROSTOPIC": "/light_color",
})
