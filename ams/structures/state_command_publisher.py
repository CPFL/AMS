#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


STATE_COMMAND_PUBLISHER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "StateCommandPublisher",
    "STATE_CMD": {
        "MAIN": {
            "START": 1,
            "INIT": 2,
        },
        "SUB": {
            "KEEP": 13,
            "STOP": 14
        },
    },
    "ROSNODE": "ams_state_command_publisher",
    "ROSTOPIC": "/state_cmd",
})
