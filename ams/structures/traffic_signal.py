#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


TRAFFIC_SIGNAL = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "TrafficSignal",
    "TOPIC": {
        "CATEGORIES": {
            "STATUS": ["status"],
            "SCHEDULES": ["schedules"],
            "CYCLE": ["cycle"]
        }
    },
    "LOWER_LIMIT_RATE": 600.0,  # [sec]
    "STATE": {
        "GREEN": "green",
        "YELLOW": "yellow",
        "RED": "red",
        "UNKNOWN": "unknown"
    }
})
