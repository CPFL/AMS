#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator

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


class Location(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)