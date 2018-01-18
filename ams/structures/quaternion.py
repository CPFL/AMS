#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

template = {
    "w": 0.0,
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
}

schema = {
    "w": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
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


class Quaternion(get_base_class(template, schema)):
    pass
