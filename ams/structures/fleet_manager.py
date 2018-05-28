#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


FLEET_MANAGER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "FleetManager",
    "TOPIC": {
        "CATEGORIES": {
            "STATUS": ["status"],
            "SCHEDULES": ["schedules"]
        }
    },
    "TRIGGER": {
        "LOG_IN": "log_in",
        "LOG_OUT": "log_out"
    },
    "STATE": {
        "LOG_IN": "log_in",
        "LOG_OUT": "log_out"
    }
})
