#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


topic = {
    "LANE_ARRAY": "/based/lane_waypoints_raw"
}

const = {
    "NODE_NAME": "ros",
    "ROLE_NAME": "ros",
    "TOPIC": topic
}

CONST = get_namedtuple_from_dict("CONST", const)
