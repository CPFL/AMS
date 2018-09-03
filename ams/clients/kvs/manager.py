#!/usr/bin/env python
# coding: utf-8

from copy import copy, deepcopy
import json
import traceback
from multiprocessing import Manager

from ams import AttrDict, logger
from ams.helpers import Kvs
from ams.structures import CLIENT


def compare_keys(key, key_pattern):
    if key_pattern == key:
        return True
    else:
        pattern_parts = key_pattern.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)
        key_parts = key.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[:-1]
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


class KVSClient(object):

    CONST = CLIENT.KVS.BASE_CLIENTS.MULTIPROCESSING

    def __init__(self):
        self.__manager = Manager()
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
        latest_key = Kvs.delete_old_keys_and_get_latest_key(keys, self.__d.pop)
        self.__key_relation[key] = latest_key

        try:
            value = deepcopy(self.__d[latest_key])
        except KeyError:
            value = None

        return value

    def set(self, key, value, get_key=None, timestamp_string=None):
        if get_key is None:
            if timestamp_string is None:
                timestamp_string = Kvs.get_timestamp_string()
            timestamped_key = CLIENT.KVS.KEY_PATTERN_DELIMITER.join([key, timestamp_string])
        else:
            timestamp_string = Kvs.get_key_timestamp(self.__key_relation[get_key])
            timestamped_key = CLIENT.KVS.KEY_PATTERN_DELIMITER.join([key, timestamp_string])

        keys = list(filter(lambda x: key == x[:len(key)] and x[len(key)+1:].isdigit(), self.__d.keys()))
        latest_key = Kvs.delete_old_keys_and_get_latest_key(keys, self.__d.pop)

        set_flag = False
        if None in [latest_key, self.__key_relation.get(get_key, None)] or \
                int(Kvs.get_key_timestamp(latest_key)) < \
                int(Kvs.get_key_timestamp(self.__key_relation[get_key])):
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
                int(Kvs.get_key_timestamp(latest_key)),
                int(Kvs.get_key_timestamp(self.__key_relation[get_key]))
            ))
        return set_flag

    def delete(self, key):
        keys = list(filter(lambda x: key == x[:len(key)] and x[len(key) + 1:].isdigit(), self.__d.keys()))
        for k in keys:
            try:
                self.__d.pop(k)
            except KeyError:
                pass

    def keys(self, pattern="*"):
        return list(filter(lambda x: compare_keys(x, pattern), self.__d.keys()))

    @staticmethod
    def disconnect():
        logger.warning("Not supported this method.")

