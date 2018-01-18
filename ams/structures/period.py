#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator

template = {
    "start": 0,
    "end": 0,
}

schema = {
    "start": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "end": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Period(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)
