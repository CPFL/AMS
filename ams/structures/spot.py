#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Location, Polygon, Events, Targets


template = {
    "targets": Targets.get_template(),
    "contact": Location.get_template(),
    "polygon": Polygon.get_template(),
    "events": Events.get_template(),
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
    "events": Events.get_schema()
}


class Spot(get_structure_superclass(template, schema)):
    pass
