#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


VEHICLE = get_namedtuple_from_dict("CONST", {
    "TOPIC": {
        "PUBLISH": "pub_vehicle",
        "SUBSCRIBE": "sub_vehicle",
        "SCHEDULES": "/schedules",
    },
    "GEO": {
        "GROUP": "Vehicle",
        "TOPIC": {
            "PUBLISH": "pub_geo_vehicle",
            "SUBSCRIBE": "sub_geo_vehicle",
        },
    },
    "STATE": {
        "LOG_IN": "logIn",
        "MOVE": "move",
        "STOP": "stop",
        "LOG_OUT": "logOut",
        "WILL": "will"
    },
    "ACTION": {
        "MOVE": "move",
        "STOP": "stop"
    }
})
