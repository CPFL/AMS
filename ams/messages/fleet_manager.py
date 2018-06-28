#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass


status_template = {
    "name": "fm0",
    "time": 0.0,
    "state": "default",
    "relations": {"from_id": ["to_id"]}
}

status_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "state": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "relations": {
        "type": "dict",
        "required": True,
        "nullable": True,
        "keyschema": {
            "type": "string"
        },
        "valueschema": {
            "type": "list",
            "valueschema": {
                "schema": {
                    "type": "string",
                    "required": True,
                    "nullable": False
                },
            },
            "minlength": 1
        }
    }
}


class Status(get_structure_superclass(status_template, status_schema)):
    pass
