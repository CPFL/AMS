#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


FLEET_MANAGER = get_namedtuple_from_dict("CONST", {
    "TOPIC": {
        "PUBLISH": "pub_fleet_manager",
        "SUBSCRIBE": "sub_fleet_manager",
        "CATEGORIES": {
            "STATUS": ["status"],
            "SCHEDULES": ["schedules"]
        }
    },
    "ACTION": {
        "PUBLISH_RELATIONS": "pub_relations"
    },
    "STATE": {
        "STAND_BY": "standby",
        "RUNNING": "running"
    }
})
