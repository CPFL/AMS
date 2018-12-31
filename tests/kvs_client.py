#!/usr/bin/env python
# coding: utf-8

import unittest
import traceback

from ams.clients import get_kvs_client_class


def generate_manager_client():
    import multiprocessing
    kvs_client = get_kvs_client_class(multiprocessing)()
    return kvs_client


def generate_redis_client(host):
    try:
        import redis
        kvs_client = get_kvs_client_class(redis)()
        kvs_client.set_args_of_ConnectionPool(host=host)
        kvs_client.set_args_of_StrictRedis()
        return kvs_client
    except ImportError:
        traceback.print_exc()
        return None


class Test(unittest.TestCase):

    def __init__(self, method_name):

        super(Test, self).__init__(method_name)
        self.manager_client_for_test_connect = generate_manager_client()

        self.manager_client_for_test_set = generate_manager_client()
        if self.manager_client_for_test_set is not None:
            self.manager_client_for_test_set.connect()

        self.manager_client_for_test_get = generate_manager_client()
        if self.manager_client_for_test_get is not None:
            self.manager_client_for_test_get.connect()
            key = "/test/get/manager"
            value = "manager_data"
            self.manager_client_for_test_get.set(key, value)

        self.manager_client_for_test_delete = generate_manager_client()
        if self.manager_client_for_test_delete is not None:
            self.manager_client_for_test_delete.connect()
            key = "/test/delete/manager"
            value = "manager_data"
            self.manager_client_for_test_delete.set(key, value)

        self.manager_client_for_test_keys = generate_manager_client()
        if self.manager_client_for_test_keys is not None:
            self.manager_client_for_test_keys.connect()
            key = "/test/keys/manager/1"
            value = "manager_data"
            self.manager_client_for_test_keys.set(key, value)
            key = "/test/keys/manager/2"
            self.manager_client_for_test_keys.set(key, value)

        self.redis_client_for_test_connect = generate_redis_client("localhost")

        self.redis_client_for_test_set = generate_redis_client("localhost")
        if self.redis_client_for_test_set is not None:
            self.redis_client_for_test_set.connect()

        self.redis_client_for_test_get = generate_redis_client("localhost")
        if self.redis_client_for_test_get is not None:
            self.redis_client_for_test_get.connect()
            key = "/test/get/redis"
            value = "redis_data"
            self.redis_client_for_test_get.set(key, value)

        self.redis_client_for_test_delete = generate_redis_client("localhost")
        if self.redis_client_for_test_delete is not None:
            self.redis_client_for_test_delete.connect()
            key = "/test/delete/redis"
            value = "redis_data"
            self.redis_client_for_test_delete.set(key, value)

        self.redis_client_for_test_keys = generate_redis_client("localhost")
        if self.redis_client_for_test_keys is not None:
            self.redis_client_for_test_keys.connect()
            key = "/test/keys/redis/1"
            value = "redis_data"
            self.redis_client_for_test_keys.set(key, value)
            key = "/test/keys/redis/2"
            self.redis_client_for_test_keys.set(key, value)

    def test_connect(self):
        if self.manager_client_for_test_connect is not None:
            self.manager_client_for_test_connect.connect()

        if self.redis_client_for_test_connect is not None:
            self.redis_client_for_test_connect.connect()

    def test_set(self):
        if self.manager_client_for_test_set is not None:
            key = "/test/set/manager"
            value = "manager_data"
            ret = self.manager_client_for_test_set.set(key, value)
            self.assertEqual(True, ret)
            self.assertEqual(value, self.manager_client_for_test_set.get(key))

        if self.redis_client_for_test_set is not None:
            key = "/test/set/redis"
            value = "manager_data"
            ret = self.redis_client_for_test_set.set(key, value)
            self.assertEqual(True, ret)
            self.assertEqual(value, self.redis_client_for_test_set.get(key))

    def test_get(self):
        if self.manager_client_for_test_get is not None:
            key = "/test/get/manager"
            value = "manager_data"
            self.assertEqual(value, self.manager_client_for_test_get.get(key))

        if self.redis_client_for_test_get is not None:
            key = "/test/set/redis"
            value = "manager_data"
            self.assertEqual(value, self.redis_client_for_test_get.get(key))

    def test_delete(self):
        if self.manager_client_for_test_delete is not None:
            key = "/test/delete/manager"
            ret = self.manager_client_for_test_delete.delete(key)
            self.assertEqual(None, ret)
            self.assertEqual(None, self.manager_client_for_test_delete.get(key))

        if self.redis_client_for_test_delete is not None:
            key = "/test/delete/redis"
            ret = self.redis_client_for_test_delete.delete(key)
            self.assertEqual(None, ret)
            self.assertEqual(None, self.redis_client_for_test_delete.get(key))

    def test_keys(self):
        if self.manager_client_for_test_keys is not None:
            key = "/test/keys/manager/*"
            value = self.manager_client_for_test_keys.keys(key)
            expected = ["/test/keys/manager/1", "/test/keys/manager/2"]
            self.assertEqual(len(expected), len(value))
            self.assertEqual(set(expected), set(value))

        if self.redis_client_for_test_keys is not None:
            key = "/test/keys/redis/*"
            value = self.redis_client_for_test_keys.keys(key)
            expected = ["/test/keys/redis/1", "/test/keys/redis/2"]
            self.assertEqual(len(expected), len(value))
            self.assertEqual(set(expected), set(value))
