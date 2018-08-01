#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict
from ams.nodes.dispatcher.const import const as dispatcher_const

topic = {
    "CATEGORIES": {}
}
topic["CATEGORIES"].update(dispatcher_const["TOPIC"]["CATEGORIES"])

event = {}
event.update(dispatcher_const["EVENT"])

state = {}
state.update(dispatcher_const["STATE"])

transportation_state = {}
transportation_state.update(dispatcher_const["TRANSPORTATION"]["STATE"])

const = {
    "NODE_NAME": "autoware_dispatcher",
    "ROLE_NAME": "dispatcher",
    "TOPIC": topic,
    "EVENT": event,
    "STATE": state,
    "TRANSPORTATION": {
        "STATE": transportation_state
    }
}

CONST = get_namedtuple_from_dict("CONST", const)
