#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Pose, Location, Schedule

status_template = {
    "name": "v0",
    "time": 0.0,
    "state": "default",
    "schedule": Schedule.get_template(),
    "location": Location.get_template(),
    "pose": Pose.get_template()
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
    "location": {
        "type": "dict",
        "schema": Location.get_schema(),
        "required": True,
        "nullable": True,
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": True,
        "nullable": True,
    }
}


class Status(get_base_class(status_template, status_schema)):
    pass
