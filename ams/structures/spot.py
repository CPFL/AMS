#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Location, Polygon, Schedules, Targets

template = {
    "targets": Targets.get_template(),
    "contact": Location.get_template(),
    "polygon": Polygon.get_template(),
    "schedules": Schedules.get_template(),
}

schema = {
    "targets": Targets.get_schema(),
    "contact": {
        "schema": Location.get_schema(),
        "required": True,
        "nullable": False,
    },
    "polygon": {
        "schema": Polygon.get_schema(),
        "required": True,
        "nullable": True,
    },
    "schedules": Schedules.get_schema()
}


class Spot(get_base_class(template, schema)):
    pass
