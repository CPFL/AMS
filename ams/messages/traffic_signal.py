#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Schedule

status = {
    "route_code": "0:0_1:1",
    "time": 0.0,
    "state": "default"
}

status_schema = {
    "route_code": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "state": {
        "type": "string",
        "required": True,
        "nullable": False,
    }
}


class Status(get_base_class(status, status_schema)):
    pass


schedules = [Schedule.get_template()]

schedules_schema = {
    "type": "list",
    "valueschema": {
        "schema": Schedule.get_schema(),
        "minlength": 1
    }
}


class Schedules(get_base_class(schedules, schedules_schema)):
    pass
