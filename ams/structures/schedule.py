#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Targets, Period, Route


schedule_template = {
    "id": "uuid",
    "event": "default",
    "period": Period.get_template(),
    "route_code": "0:0>1:1",
    "targets": Targets.get_template(),
}

schedule_schema = {
    "id": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "event": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "period": {
        "type": "dict",
        "schema": Period.get_schema(),
        "required": False,
        "nullable": True,
    },
    "route_code": {
        "type": "string",
        "required": False,
        "nullable": False,
    },
    "targets": Targets.get_schema(),
}


class Schedule(get_structure_superclass(schedule_template, schedule_schema)):
    pass


schedules_template = [Schedule.get_template()]

schedules_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": Schedule.get_schema(),
        "required": True,
        "nullable": False,
    },
    "nullable": False,
    "minlength": 0
}


class Schedules(get_structure_superclass(schedules_template, schedules_schema)):
    pass
