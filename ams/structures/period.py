#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

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


class Period(get_base_class(template, schema)):
    pass
