#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Target, Period, Route

template = {
    "targets": [Target.get_template()],
    "event": "default",
    "period": Period.get_template(),
    "route": Route.get_template(),
}

schema = {
    "targets": {
        "type": "list",
        "required": True,
        "nullable": False,
        "valueschema": {
            "schema": Target.get_schema(),
            "minlength": 1
        }
    },
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
        "nullable": True,
    }
}


class Schedule(get_base_class(template, schema)):
    pass
