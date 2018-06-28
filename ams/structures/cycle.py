#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass


template = {
    "time": 0.0,
    "period": 1.0,
    "phases": [{
        "state": "default",
        "duration": 0.1
    }]
}

schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "period": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "phases": {
        "type": "list",
        "valueschema": {
            "schema": {
                "state": {
                    "type": "string",
                    "required": True,
                    "nullable": False
                },
                "duration": {
                    "type": "number",
                    "required": True,
                    "nullable": False
                }
            },
        },
        "required": True,
        "nullable": False,
        "minlength": 1
    }
}


class Cycle(get_structure_superclass(template, schema)):
    pass
