#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


USER = get_namedtuple_from_dict("CONST", {
    "TOPIC": {
        "CATEGORIES": {
            "STATUS": ["status"],
        }
    },
    "EVENT": {
        "TRIP": "trip",
    },
    "TRIGGER": {
        "LOG_IN": "log_in",
        "LOG_OUT": "log_out"
    },
    "STATE": {
        "LOG_IN": "log_in",
        "LOG_OUT": "log_out"
    }
})
