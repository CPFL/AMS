#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator

template = {
    "w": 0.0,
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
}

schema = {
    "w": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "x": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "y": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "z": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Quaternion(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)
