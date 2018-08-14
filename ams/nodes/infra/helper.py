#!/usr/bin/env python
# coding: utf-8

from ams.structures import KVS_CLIENT
from ams.helpers import Target, Schedule
from ams.nodes.infra import CONST, Structure


class Helper(object):

    INFRA = CONST
    Structure = Structure

    get_id = Schedule.get_id
    get_time = Schedule.get_time

    @classmethod
    def get_config_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join(
            [Target.get_code(target_roles[cls.INFRA.ROLE_NAME])] +
            cls.INFRA.TOPIC.CATEGORIES.CONFIG)

    @classmethod
    def get_status_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join(
            [Target.get_code(target_roles[cls.INFRA.ROLE_NAME])] +
            cls.INFRA.TOPIC.CATEGORIES.STATUS)

    @classmethod
    def get_schedules_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles[cls.INFRA.ROLE_NAME])] +
            cls.INFRA.TOPIC.CATEGORIES.SCHEDULES)

    @classmethod
    def get_config_key_and_value(cls, clients, target_roles):
        key = cls.get_config_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Config.new_data(**value)
        return key, value

    @classmethod
    def get_status_key_and_value(cls, clients, target_roles):
        key = cls.get_status_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Status.new_data(**value)
        return key, value

    @classmethod
    def get_schedules_key_and_value(cls, clients, target_roles):
        key = cls.get_schedules_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "list":
            value = Schedule.new_schedules(value)
        return key, value

    @classmethod
    def get_schedule_events(cls, schedules):
        return list(map(lambda x: x.event, schedules))

    @classmethod
    def get_next_schedule_index(cls, status, schedules):
        next_schedule = Schedule.get_next_schedule_by_current_schedule_id(
            schedules, status.schedule_id)
        return schedules.index(next_schedule)

    @classmethod
    def get_next_schedule_id(cls, status, schedules):
        return Schedule.get_next_schedule_by_current_schedule_id(schedules, status.schedule_id).id

    @classmethod
    def set_config(cls, clients, target_roles, config):
        key = cls.get_config_key(target_roles)
        return clients["kvs"].set(key, config)

    @classmethod
    def set_schedules(cls, clients, target_roles, schedules, timestamp_string=None):
        key = cls.get_schedules_key(target_roles)
        return clients["kvs"].set(key, schedules, timestamp_string=timestamp_string)

    @classmethod
    def set_status(cls, clients, target_roles, status, schedules_key=None):
        key = cls.get_status_key(target_roles)
        return clients["kvs"].set(key, status, schedules_key)

    @classmethod
    def activation_timeout(cls, status):
        return cls.INFRA.ACTIVATION_REQUEST_TIMEOUT < cls.get_time() - status.updated_at

    @classmethod
    def update_and_set_status(cls, clients, target_roles, status, new_state, schedules=None, schedules_key=None):
        status.state = new_state
        status.updated_at = cls.get_time()
        if schedules is not None:
            status.schedule_id = cls.get_next_schedule_id(status, schedules)
        return cls.set_status(clients, target_roles, status, schedules_key)
