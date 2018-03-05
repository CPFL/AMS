#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


DECISION_MAKER_STATES_SUBSCRIBER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "DecisionMakerStatesSubscriber",
    "DECISION_MAKER_STATES": {
        "MAIN_STATE": {
            "INITIAL": "Initial",
            "MISSION_COMPLETE": "MissionComplete",
        },
        "ACC_STATE": {
            "KEEP": "Keep",
            "STOP": "Stop",
        },
        "STR_STATE": {
            "NONE": ""
        },
        "BEHAVIOR_STATE": {
            "TRAFFIC_LIGHT_GREEN": "\nTrafficLightGreen",
            "TRAFFIC_LIGHT_RED": "\nTrafficLightRed"
        }
    },
    "ROSNODE": "ams_decision_maker_subscriber",
    "ROSTOPIC": "/decisionmaker/states",
    "TOPIC_CATEGORIES": ["decision_maker_states"],
})
