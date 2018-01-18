#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

template = {
    "waypoint_id": "0",
    "arrow_code": "0_1",
    "geohash": "123456789012345",
}

schema = {
    "waypoint_id": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "arrow_code": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "geohash": {
        "type": "string",
        "required": True,
        "nullable": False,
        "minlength": 15,
    }
}


class Location(get_base_class(template, schema)):
    pass
