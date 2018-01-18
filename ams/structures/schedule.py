#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Period, Route

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


class Schedule(get_base_class(template, schema)):
    pass
