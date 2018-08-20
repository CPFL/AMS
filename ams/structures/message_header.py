#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass


message_header_template = {
    "id": "message_id",
    "time": 0.0,
    "request_id": "request_message_id",
    "version": 0.3
}

message_header_schema = {
    "id": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "request_id": {
        "type": "string",
        "required": False,
        "nullable": True
    },
    "version": {
        "type": "string",
        "required": True,
        "nullable": False
    }
}


class MessageHeader(get_structure_superclass(message_header_template, message_header_schema)):
    pass
