#!/usr/bin/env python
# coding: utf-8

from ams import VERSION
from ams.structures import CLIENT
from ams.helpers import Target, Schedule
from ams.nodes.base import CONST, Structure, Message


class Helper(object):

    CONST = CONST
    Structure = Structure
    Message = Message

    get_id = Schedule.get_id
    get_time = Schedule.get_time

    @classmethod
    def get_config_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles[cls.CONST.ROLE_NAME]),
            "config"])

    @classmethod
    def get_status_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles[cls.CONST.ROLE_NAME]),
            "status"])

    @classmethod
    def get_config(cls, clients, target_roles):
        key = cls.get_config_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Config.new_data(**value)
        return value

    @classmethod
    def get_status(cls, clients, target_roles):
        key = cls.get_status_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Status.new_data(**value)
        return value

    @classmethod
    def get_response_config_message(cls, clients, target_roles, request_message):
        return cls.Message.Config.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION,
                "request_id": request_message.header.id
            },
            "body": cls.get_config(clients, target_roles)
        })

    @classmethod
    def get_response_status_message(cls, clients, target_roles, request_message):
        return cls.Message.Status.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION,
                "request_id": request_message.header.id
            },
            "body": cls.get_status(clients, target_roles)
        })

    @classmethod
    def set_config(cls, clients, target_roles, value, get_key=None, timestamp_string=None):
        key = cls.get_config_key(target_roles)
        return clients["kvs"].set(key, value, get_key, timestamp_string)

    @classmethod
    def set_status(cls, clients, target_roles, value, get_key=None, timestamp_string=None):
        key = cls.get_status_key(target_roles)
        return clients["kvs"].set(key, value, get_key, timestamp_string)
