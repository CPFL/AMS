#!/usr/bin/env python
# coding: utf-8

from copy import copy, deepcopy
import json
import traceback

import redis

from ams import AttrDict, logger
from ams.helpers import Kvs
from ams.structures import CLIENT


class ArgsSetters(object):

    CONST = CLIENT.KVS.BASE_CLIENTS.REDIS

    def __init__(self):
        self.args = AttrDict()

    def set_args_of_ConnectionPool(
            self, host="localhost", port=6379, max_connections=None,
            **connection_kwargs
    ):
        connection_kwargs["host"] = connection_kwargs["host"] if "host" in connection_kwargs else host
        connection_kwargs["port"] = connection_kwargs["port"] if "port" in connection_kwargs else port
        connection_kwargs["max_connections"] = \
            connection_kwargs["max_connections"] if "max_connections" in connection_kwargs else max_connections
        connection_kwargs.update({"host": host, "port": port, })
        self.args.connection_pool = connection_kwargs

    def set_args_of_StrictRedis(
            self, host='localhost', port=6379,
            db=0, password=None, socket_timeout=None,
            socket_connect_timeout=None,
            socket_keepalive=None, socket_keepalive_options=None,
            connection_pool=None, unix_socket_path=None,
            encoding="utf-8", encoding_errors='strict',
            charset=None, errors=None,
            decode_responses=False, retry_on_timeout=False,
            ssl=False, ssl_keyfile=None, ssl_certfile=None,
            ssl_cert_reqs=None, ssl_ca_certs=None,
            max_connections=None
    ):
        self.args.strict_redis = copy(locals())
        self.args.strict_redis.pop("self")


def get_keys(client, base_key):
    return list(filter(
        lambda x: base_key == x[:len(base_key)] and x[len(base_key) + 1:].isdigit(),
        map(
            lambda x: x.decode("utf-8"),
            client.keys(CLIENT.KVS.KEY_PATTERN_DELIMITER.join([base_key, "*"]))
        )
    ))


class KVSClient(ArgsSetters):
    def __init__(self):
        super().__init__()
        self.__connection_pool = None
        self.__client = None
        self.__key_relation = {}

    def connect(self):
        self.__connection_pool = redis.ConnectionPool(**self.args.connection_pool)
        self.args.strict_redis.connection_pool = self.__connection_pool
        self.__client = redis.StrictRedis(**self.args.strict_redis)

    def get(self, key):
        keys = get_keys(self.__client, key)
        latest_key = Kvs.delete_old_keys_and_get_latest_key(keys, self.__client.delete)
        self.__key_relation[key] = latest_key
        binary_value = self.__client.get(key if latest_key is None else latest_key)
        return binary_value if binary_value is None else json.loads(binary_value.decode("utf-8"))

    def set(self, key, value, get_key=None, timestamp_string=None):
        if get_key is None:
            if timestamp_string is None:
                timestamp_string = Kvs.get_timestamp_string()
            timestamped_key = CLIENT.KVS.KEY_PATTERN_DELIMITER.join([key, timestamp_string])
        else:
            timestamp_string = Kvs.get_key_timestamp(self.__key_relation[get_key])
            timestamped_key = CLIENT.KVS.KEY_PATTERN_DELIMITER.join([key, timestamp_string])

        keys = get_keys(self.__client, key)
        latest_key = Kvs.delete_old_keys_and_get_latest_key(keys, self.__client.delete)

        set_flag = False
        if None in [latest_key, self.__key_relation.get(get_key, None)] or \
                int(Kvs.get_key_timestamp(latest_key)) < \
                int(Kvs.get_key_timestamp(self.__key_relation[get_key])):
            set_flag = self.__client.setnx(timestamped_key, json.dumps(value))
        return set_flag

    def delete(self, key):
        keys = get_keys(self.__client, key)
        for k in keys:
            self.__client.delete(k)

    def keys(self, pattern="*"):
        return list(map(lambda x: x.decode("utf-8"), self.__client.keys(pattern)))

    def disconnect(self):
        if self.args.strict_redis.connection_pool is not None:
            self.__client.connection_pool.disconnect()
