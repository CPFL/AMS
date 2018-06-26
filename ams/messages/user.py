#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Schedule


status_template = {
    "name": "u0",
    "time": 0.0,
    "trip_schedules": [Schedule.get_template()],
    "state": "default",
    "schedule": Schedule.get_template(),
}

status_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "trip_schedules": {
        "type": "list",
        "valueschema": {
            "schema": Schedule.get_schema(),
            "minlength": 1
        },
        "required": True,
        "nullable": True
    },
    "state": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "schedule": {
        "type": "dict",
        "schema": Schedule.get_schema(),
        "required": True,
        "nullable": True,
    },
}


class Status(get_structure_superclass(status_template, status_schema)):
    pass
