#!/usr/bin/env python
# coding: utf-8

import importlib

from ams import get_namedtuple_from_dict

base_clients = {
    "MULTIPROCESSING": {
        "MODULE_NAME": "multiprocessing"
    }
}

if importlib.util.find_spec("redis") is not None:
    base_clients["REDIS"] = {
        "MODULE_NAME": "redis",
    }

DICT_CLIENT = get_namedtuple_from_dict("CONST", {
    "KEY_PATTERN_DELIMITER": "_",
    "BASE_CLIENTS": base_clients
})
