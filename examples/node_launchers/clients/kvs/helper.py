#!/usr/bin/env python
# coding: utf-8

import multiprocessing
from ams import logger, get_ams_kvs_client_class
from ams.structures import KVS_CLIENT


def print_all(kvs_client):
    keys = kvs_client.keys("*")
    for stamped_key in keys:
        key = KVS_CLIENT.KEY_PATTERN_DELIMITER.join(stamped_key.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[:-1])
        logger.pp({"kvs": {key: kvs_client.get(key)}})
