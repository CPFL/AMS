#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

message = {
    "time": 0.0,
    "event": "default",  # start, check, kill, ok
    "pid": 0
}


message_schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "event": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "pid": {
        "type": "integer",
        "required": True,
        "nullable": False
    }
}


class Message(get_base_class(message, message_schema)):
    pass
