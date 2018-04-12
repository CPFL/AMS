#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


SIM_TAXI_FLEET = get_namedtuple_from_dict("CONST", {
    "DISPATCHABLE_GEOHASH_DIGIT": 6,
    "TIMEOUT": 30.0,
    "TRIGGER": {
        "WAIT_USER_REQUEST": "wait_user_request",
        "DISPATCH": "dispatch",
        "NOTICE": "notice",
        "WAIT_TAXI_ARRIVAL": "wait_taxi_arrival",
        "WAIT_USER_ACTION": "wait_user_action",
    },
    "STATE": {
        "WAITING_FOR_USER_LOG_IN": "waiting_for_user_log_in",
        "WAITING_FOR_USER_REQUEST": "waiting_for_user_request",
        "WAITING_FOR_TAXI_ARRIVE_AT_USER_LOCATION": "waiting_for_taxi_arrive_at_user_location",
        "WAITING_FOR_USER_GETTING_ON": "waiting_for_user_getting_on",
        "WAITING_FOR_TAXI_ARRIVE_AT_USER_DESTINATION": "waiting_for_taxi_arrive_at_user_destination",
        "WAITING_FOR_USER_GETTING_OUT": "waiting_for_user_getting_out",
    }
})
