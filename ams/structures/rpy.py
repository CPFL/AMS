#!/usr/bin/env python
# coding: utf-8

from numpy import pi
from ams.structures import get_base_class

pi2 = 2.0*pi

template = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}

schema = {
    "roll": {
        "type": "number",
        "required": True,
        "nullable": False,
        "min": 0.0,
        "max": pi2
    },
    "pitch": {
        "type": "number",
        "required": True,
        "nullable": False,
        "min": 0.0,
        "max": pi2
    },
    "yaw": {
        "type": "number",
        "required": True,
        "nullable": False,
        "min": 0.0,
        "max": pi2
    }
}


class Rpy(get_base_class(template, schema)):
    pass
