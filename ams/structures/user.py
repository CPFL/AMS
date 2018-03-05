#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


USER = get_namedtuple_from_dict("CONST", {
    "TOPIC": {
        "PUBLISH": "pub_user",
        "SUBSCRIBE": "sub_user",
        "GEO": {
            "PUBLISH": "pub_geo_vehicle",
            "SUBSCRIBE": "sub_geo_vehicle"
        },
        "CATEGORIES": {
            "STATUS": ["status"],
        }
    },
    "STATE": {
        "LOG_IN": "login",
        "LOG_OUT": "logout"
    },
    "ACTION": {
        "REQUEST": "request"
    }
})
