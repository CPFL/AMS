#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator
from ams.structures import Position, Orientation

template = {
    "position": Position.get_template(),
    "orientation": Orientation.get_template()
}

schema = {
    "position": {
        "type": "dict",
        "schema": Position.get_schema(),
        "required": True,
        "nullable": False,
    },
    "orientation": {
        "type": "dict",
        "schema": Orientation.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class Pose(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)
