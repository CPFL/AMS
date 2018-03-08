#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


DECISION_MAKER_STATES_SUBSCRIBER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "DecisionMakerStatesSubscriber",
    "MAIN": {
        "INITIAL": "Initial",
        "MISSION_COMPLETE": "MissionComplete",
    },
    "ACC": {
        "KEEP": "Keep",
        "STOP": "Stop",
    },
    "STR": {
        "NONE": ""
    },
    "BEHAVIOR": {
        "TRAFFIC_LIGHT_GREEN": "\nTrafficLightGreen",
        "TRAFFIC_LIGHT_RED": "\nTrafficLightRed",
        "WAIT_ORDERS": "\nWaitOrders",
    },
    "ROSNODE": "ams_decision_maker_subscriber",
    "ROSTOPIC": "/decisionmaker/states",
    "TOPIC_CATEGORIES": ["decision_maker_states"],
})
