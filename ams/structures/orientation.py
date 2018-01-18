#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator
from ams.structures import Rpy, Quaternion

template = {
    "rpy": Rpy.get_template(),
    "quaternion": Quaternion.get_template()
}

schema = {
    "rpy": {
        "type": "dict",
        "schema": Rpy.get_schema(),
        "required": True,
        "nullable": False,
    },
    "quaternion": {
        "type": "dict",
        "schema": Quaternion.get_schema(),
        "required": False,
        "nullable": False,
    }
}


class Orientation(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)
