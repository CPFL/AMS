#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Targets, Period, Route


event_template = {
    "id": "uuid",
    "name": "default",
    "period": Period.get_template(),
    "route_code": "0:0>1:1",
    "targets": Targets.get_template(),
}

event_schema = {
    "id": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "name": {
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


class Event(get_structure_superclass(event_template, event_schema)):
    pass


events_template = [Event.get_template()]

events_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": Event.get_schema(),
        "required": True,
        "nullable": False,
    },
    "nullable": False,
    "minlength": 0
}


class Events(get_structure_superclass(events_template, events_schema)):
    pass
