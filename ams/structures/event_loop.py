#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


EVENT_LOOP = get_namedtuple_from_dict("CONST", {
    "KEEP_ALIVE": 60,
    "TOPIC": {
        "PUBLISH": "pub_event_loop",
        "SUBSCRIBE": "sub_event_loop"
    },
    "ACTION": {
        "CHECK": "check",
        "KILL": "kill"
    },
    "STATE": {
        "START": "start",
        "WILL": "will",
        "DISCONNECT": "disconnect"
    },
    "RESPONSE": {
        "OK": "ok"
    }
})

