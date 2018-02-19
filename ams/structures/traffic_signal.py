#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


TRAFFIC_SIGNAL = get_namedtuple_from_dict("CONST", {
    "LOWER_LIMIT_RATE": 600.0,  # [sec]
    "STATE": {
        "GREEN": "green",
        "YELLOW": "yellow",
        "RED": "red",
        "UNKNOWN": "unknown"
    },
    "TOPIC": {
        "PUBLISH": "pub_traffic_signal",
        "SUBSCRIBE_SCHEDULES": "sub_traffic_signal_schedules",
        "SUBSCRIBE_CYCLE": "sub_traffic_signal_cycle"
    }
})
