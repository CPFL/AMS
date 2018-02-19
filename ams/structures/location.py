#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

template = {
    "geohash": "123456789012345",
    "waypoint_id": "0",
    "arrow_code": "0_1",
}

schema = {
    "geohash": {
        "type": "string",
        "required": True,
        "nullable": False,
        "minlength": 15,
    },
    "waypoint_id": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "arrow_code": {
        "type": "string",
        "required": True,
        "nullable": True,
    }
}


class Location(get_base_class(template, schema)):
    pass
