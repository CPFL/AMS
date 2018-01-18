#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator
from ams.structures import Period, Route

template = {
    "event": None,
    "period": Period.get_template(),
    "route": Route.get_template(),
}

schema = {
    "event": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "period": {
        "type": "dict",
        "schema": Period.get_schema(),
        "required": True,
    },
    "route": {
        "type": "dict",
        "schema": Route.get_schema(),
        "required": False,
    }
}


class Schedule(Validator):
    def __init__(self):
        self.__template = template
        super().__init__(schema)

    @staticmethod
    def get_template():
        return deepcopy(template)

    @staticmethod
    def get_schema():
        return deepcopy(schema)
