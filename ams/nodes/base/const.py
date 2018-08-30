#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


topic = {
    "CATEGORIES": {
        "REQUEST_GET_CONFIG": ["request", "get", "config"],
        "RESPONSE_GET_CONFIG": ["response", "get", "config"],
        "REQUEST_GET_STATUS": ["request", "get", "status"],
        "RESPONSE_GET_STATUS": ["response", "get", "status"],
    }
}

const = {
    "NODE_NAME": "base",
    "ROLE_NAME": "base",
    "TOPIC": topic
}

CONST = get_namedtuple_from_dict("CONST", const)
