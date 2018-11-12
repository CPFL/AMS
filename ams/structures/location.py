#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass


location_template = {
    "geohash": "123456789012345",
    "waypoint_id": "0",
    "lane_code": "0_1",
}

location_schema = {
    "geohash": {
        "type": "string",
        "required": False,
        "nullable": True,
        "minlength": 15,
    },
    "waypoint_id": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "lane_code": {
        "type": "string",
        "required": True,
        "nullable": True,
    }
}


class Location(get_structure_superclass(location_template, location_schema)):
    pass


locations_template = [Location.get_template()]

locations_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": Location.get_schema(),
        "required": True,
        "nullable": False,
    },
    "nullable": True,
    "minlength": 1
}


class Locations(get_structure_superclass(locations_template, locations_schema)):
    pass
