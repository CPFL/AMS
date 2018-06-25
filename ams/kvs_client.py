#!/usr/bin/env python
# coding: utf-8

from copy import copy, deepcopy
import json
from time import time

from ams import AttrDict, logger
from ams.structures import KVS_CLIENT


def get_timestamp_string(timestamp=None):
    if timestamp is None:
        timestamp = time()
    return str(int(1000.0*timestamp))


def get_key_timestamp(key):
    return key.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[-1]


def delete_old_keys_and_get_latest_key(keys, delete_function):
    sorted_keys = sorted(keys, key=lambda x: int(get_key_timestamp(x)))
    list(map(delete_function, sorted_keys[:-1]))
    return sorted_keys[-1] if 0 < len(sorted_keys) else None


def get_ams_kvs_client_class(base_kvs_client_module):

    if base_kvs_client_module.__name__ == KVS_CLIENT.BASE_CLIENTS.MULTIPROCESSING.MODULE_NAME:

        def compare_keys(key, key_pattern):
            if key_pattern == key:
                return True
            else:
                pattern_parts = key_pattern.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)
                key_parts = key.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[:-1]
                for i, pattern_part in enumerate(pattern_parts):
                    if pattern_part == "*":
                        return True
                    else:
                        if all([
                            pattern_part != key_parts[i],
                            pattern_part != "+",
                        ]):
                            return False
            return True

        class WrapperMethods(object):

            CONST = KVS_CLIENT.BASE_CLIENTS.MULTIPROCESSING

            def __init__(self):
                self.__manager = base_kvs_client_module.Manager()
                self.__d = self.__manager.dict()
                self.__d_lock = self.__manager.Lock()
                self.__key_relation = {}

            @staticmethod
            def connect():
                logger.warning("Not supported this method.")

            def get(self, key):

                keys = list(filter(lambda x: key == x[:len(key)] and x[len(key)+1:].isdigit(), self.__d.keys()))

                if len(keys) == 0:
                    return None
                latest_key = delete_old_keys_and_get_latest_key(keys, self.delete)
                self.__key_relation[key] = latest_key

                try:
                    value = deepcopy(self.__d[latest_key])
                except KeyError:
                    value = None

                return value

            def set(self, key, value, get_key=None, timestamp_string=None):
                if get_key is None:
                    if timestamp_string is None:
                        timestamp_string = get_timestamp_string()
                    timestamped_key = KVS_CLIENT.KEY_PATTERN_DELIMITER.join([key, timestamp_string])
                else:
                    timestamp_string = get_key_timestamp(self.__key_relation[get_key])
                    timestamped_key = KVS_CLIENT.KEY_PATTERN_DELIMITER.join([key, timestamp_string])

                keys = list(filter(lambda x: key == x[:len(key)] and x[len(key)+1:].isdigit(), self.__d.keys()))
                latest_key = delete_old_keys_and_get_latest_key(keys, self.delete)

                set_flag = False
                if None in [latest_key, self.__key_relation.get(get_key, None)] or \
                        int(get_key_timestamp(latest_key)) < int(get_key_timestamp(self.__key_relation[get_key])):
                    if timestamped_key not in self.__d.keys():
                        self.__d_lock.acquire()
                        try:
                            self.__d[timestamped_key] = deepcopy(value)
                            set_flag = True
                        except:
                            logger.warning(traceback.format_exc())
                        self.__d_lock.release()
                    else:
                        logger.warning("Failed to set for the following reasons: {} in {}".format(
                            timestamped_key,
                            list(self.__d.keys())
                        ))
                else:
                    logger.warning("Failed to set for the following reasons: None not in {} and {} >= {}".format(
                        [latest_key, self.__key_relation.get(get_key, None)],
                        int(get_key_timestamp(latest_key)),
                        int(get_key_timestamp(self.__key_relation[get_key]))
                    ))
                return set_flag

            def delete(self, key):
                try:
                    self.__d.pop(key)
                except KeyError:
                    pass

            def keys(self, pattern="*"):
                return list(filter(lambda x: compare_keys(x, pattern), self.__d.keys()))

            @staticmethod
            def disconnect():
                logger.warning("Not supported this method.")

    elif base_kvs_client_module.__name__ == KVS_CLIENT.BASE_CLIENTS.REDIS.MODULE_NAME:
        class ArgsSetters(object):

            CONST = KVS_CLIENT.BASE_CLIENTS.REDIS

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
                    client.keys(KVS_CLIENT.KEY_PATTERN_DELIMITER.join([base_key, "*"]))
                )
            ))

        class WrapperMethods(ArgsSetters):
            def __init__(self):
                super().__init__()
                self.__connection_pool = None
                self.__client = None
                self.__key_relation = {}

            def connect(self):
                self.__connection_pool = base_kvs_client_module.ConnectionPool(**self.args.connection_pool)
                self.args.strict_redis.connection_pool = self.__connection_pool
                self.__client = base_kvs_client_module.StrictRedis(**self.args.strict_redis)

            def get(self, key):
                keys = get_keys(self.__client, key)
                latest_key = delete_old_keys_and_get_latest_key(keys, self.delete)
                self.__key_relation[key] = latest_key
                binary_value = self.__client.get(key if latest_key is None else latest_key)
                return binary_value if binary_value is None else json.loads(binary_value.decode("utf-8"))

            def set(self, key, value, get_key=None, timestamp_string=None):
                if get_key is None:
                    if timestamp_string is None:
                        timestamp_string = get_timestamp_string()
                    timestamped_key = KVS_CLIENT.KEY_PATTERN_DELIMITER.join([key, timestamp_string])
                else:
                    timestamp_string = get_key_timestamp(self.__key_relation[get_key])
                    timestamped_key = KVS_CLIENT.KEY_PATTERN_DELIMITER.join([key, timestamp_string])

                keys = get_keys(self.__client, key)
                latest_key = delete_old_keys_and_get_latest_key(keys, self.delete)

                set_flag = False
                if None in [latest_key, self.__key_relation.get(get_key, None)] or \
                        int(get_key_timestamp(latest_key)) < int(get_key_timestamp(self.__key_relation[get_key])):
                    set_flag = self.__client.setnx(timestamped_key, json.dumps(value))
                return set_flag

            def delete(self, key):
                self.__client.delete(key)

            def keys(self, pattern="*"):
                return list(map(lambda x: x.decode("utf-8"), self.__client.keys(pattern)))

            def disconnect(self):
                if self.args.strict_redis.connection_pool is not None:
                    self.__client.connection_pool.disconnect()

    else:
        raise AttributeError("Unknown module {}".format(base_kvs_client_module.__name__))

    class AMSDICTClient(WrapperMethods):
        def __init__(self):
            super().__init__()

    return AMSDICTClient
