#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

template = {
    "id": "uuid",
    "node": "EventLoop"
}

schema = {
    "id": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "node": {
        "type": "string",
        "required": True,
        "nullable": False,
    }
}


class Target(get_base_class(template, schema)):
    pass
