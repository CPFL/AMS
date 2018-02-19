#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


VEHICLE = get_namedtuple_from_dict("CONST", {
    "TOPIC": {
        "PUBLISH": "pub_vehicle",
        "PUBLISH_GEO": "pub_vehicle_geo",
        "SUBSCRIBE": "sub_vehicle",
        "GEO": {
            "PUBLISH": "pub_geo_vehicle",
            "SUBSCRIBE": "sub_geo_vehicle"
        }
    },
    "STATE": {
        "MOVE": "move",
        "STOP": "stop",
        "WILL": "will"
    },
    "ACTION": {
        "MOVE": "move",
        "STOP": "stop"
    }
})
