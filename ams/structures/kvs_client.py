#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict


KVS_CLIENT = get_namedtuple_from_dict("CONST", {
    "KEY_PATTERN_DELIMITER": "/",
    "BASE_CLIENTS": {
        "MULTIPROCESSING": {
            "MODULE_NAME": "multiprocessing"
        },
        "REDIS": {
            "MODULE_NAME": "redis",
        }
    }
})
