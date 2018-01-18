#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from numpy import pi
from ams import Validator

pi2 = 2.0*pi

template = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}

schema = {
    "roll": {
        "type": "number",
        "required": True,
        "nullable": False,
        "min": 0.0,
        "max": pi2
    },
    "pitch": {
        "type": "number",
        "required": True,
        "nullable": False,
        "min": 0.0,
        "max": pi2
    },
    "yaw": {
        "type": "number",
        "required": True,
        "nullable": False,
        "min": 0.0,
        "max": pi2
    }
}


class Rpy(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)
