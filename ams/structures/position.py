#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

template = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

schema = {
    "x": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "y": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "z": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Position(get_base_class(template, schema)):
    pass
