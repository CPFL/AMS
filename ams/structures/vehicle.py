#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


VEHICLE = get_namedtuple_from_dict("CONST", {
    "TOPIC": {
        "CATEGORIES": {
            "STATUS": ["status"],
            "GEOTOPIC": ["geotopic"]
        }
    },
    "STATE": {
        "LOG_IN": "log_in",
        "LOG_OUT": "log_out"
    }
})
