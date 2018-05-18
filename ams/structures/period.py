#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass


template = {
    "start": 0,
    "end": 0,
}

schema = {
    "start": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "end": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Period(get_structure_superclass(template, schema)):
    pass
