#!/usr/bin/env python
# coding: utf-8

import redis
from ams import get_ams_kvs_client_class


def get_kvs_client(args):
    KVSClient = get_ams_kvs_client_class(redis)
    kvs_client = KVSClient()
    kvs_client.set_args_of_ConnectionPool(host=args.kvs_host)
    kvs_client.set_args_of_StrictRedis()
    return kvs_client
