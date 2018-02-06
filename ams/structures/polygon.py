#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Position, Location

template = {
    "positions": [Position.get_template()],
    "locations": [Location.get_template()]
}

schema = {
    "positions": {
        "type": "list",
        "valueschema": {
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
        "valueschema": {
            "schema": Location.get_schema(),
            "required": True,
            "nullable": False,
        },
        "required": True,
        "nullable": False,
        "minlength": 3
    }
}


class Polygon(get_base_class(template, schema)):
    pass
