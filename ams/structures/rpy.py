#!/usr/bin/env python
# coding: utf-8

from math import pi
from ams import get_structure_superclass, get_namedtuple_from_dict


RPY = get_namedtuple_from_dict("CONST", {
    "MIN": 0.0,
    "MAX": 2.0 * pi
})

template = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}

schema = {
    "roll": {
        "type": "number",
        "required": True,
        "nullable": True,
        "min": RPY.MIN,
        "max": RPY.MAX
    },
    "pitch": {
        "type": "number",
        "required": True,
        "nullable": True,
        "min": RPY.MIN,
        "max": RPY.MAX
    },
    "yaw": {
        "type": "number",
        "required": True,
        "nullable": True,
        "min": RPY.MIN,
        "max": RPY.MAX
    }
}


class Rpy(get_structure_superclass(template, schema)):
    pass
