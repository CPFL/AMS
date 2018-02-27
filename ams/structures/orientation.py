#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Rpy, Quaternion

template = {
    "rpy": Rpy.get_template(),
    "quaternion": Quaternion.get_template()
}

schema = {
    "rpy": {
        "type": "dict",
        "schema": Rpy.get_schema(),
        "required": True,
        "nullable": True,
    },
    "quaternion": {
        "type": "dict",
        "schema": Quaternion.get_schema(),
        "required": False,
        "nullable": False,
    }
}


class Orientation(get_base_class(template, schema)):
    pass
