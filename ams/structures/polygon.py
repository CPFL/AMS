#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Position, Location


template = {
    "positions": [Position.get_template()],
    "locations": [Location.get_template()]
}

schema = {
    "positions": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": Position.get_schema(),
            "required": True,
            "nullable": False,
        },
        "required": True,
        "nullable": True,
        "minlength": 3
    },
    "locations": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": Location.get_schema(),
            "required": True,
            "nullable": False,
        },
        "required": True,
        "nullable": False,
        "minlength": 3
    }
}


class Polygon(get_structure_superclass(template, schema)):
    pass
