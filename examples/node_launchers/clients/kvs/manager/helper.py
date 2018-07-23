#!/usr/bin/env python
# coding: utf-8

import multiprocessing
from ams import logger, get_ams_kvs_client_class


def get_kvs_client():
    KVSClient = get_ams_kvs_client_class(multiprocessing)
    kvs_client = KVSClient()
    return kvs_client


def print_all(kvs_client):
    keys = kvs_client.keys("*")
    for key in keys:
        logger.pp(kvs_client.get(key))
