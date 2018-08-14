#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Position, Orientation


template = {
    "position": Position.get_template(),
    "orientation": Orientation.get_template()
}

schema = {
    "position": {
        "type": "dict",
        "schema": Position.get_schema(),
        "required": True,
        "nullable": False,
    },
    "orientation": {
        "type": "dict",
        "schema": Orientation.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class Pose(get_structure_superclass(template, schema)):
    Position = Position
    Orientation = Orientation
