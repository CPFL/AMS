#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Schedules

template = {
    "key": {
        "type": "common",
        "index": 0
    },
    "common": Schedules.get_template(),
    "main": Schedules.get_template(),
    "sub": Schedules.get_template(),
}

schema = {
    "key": {
        "type": "dict",
        "schema": {
            "type": {
                "type": "string",
                "required": True,
                "nullable": False
            },
            "index": {
                "type": "integer",
                "required": True,
                "nullable": False
            }
        }
    },
    "common": Schedules.get_schema(),
    "main": Schedules.get_schema(),
    "sub": Schedules.get_schema(),
}


class ScheduleBranch(get_base_class(template, schema)):
    pass


schedule_branches = [ScheduleBranch.get_template()]

schedule_branches_schema = {
    "type": "list",
    "valueschema": {
        "schema": ScheduleBranch.get_schema(),
        "required": True,
        "nullable": False,
    },
    "minlength": 1
}


class ScheduleBranches(get_base_class(schedule_branches, schedule_branches_schema)):
    pass
