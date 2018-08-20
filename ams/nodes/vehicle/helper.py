#!/usr/bin/env python
# coding: utf-8

from ams.structures import CLIENT
from ams.helpers import Target, Schedule
from ams.nodes.vehicle import CONST, Structure


class Helper(object):

    VEHICLE = CONST
    Structure = Structure

    @classmethod
    def get_vehicle_config_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([Target.get_code(target_roles["vehicle"]), "config"])

    @classmethod
    def get_vehicle_status_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([Target.get_code(target_roles["vehicle"]), "status"])

    @classmethod
    def get_vehicle_schedules_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([Target.get_code(target_roles["vehicle"]), "schedules"])

    @classmethod
    def get_current_vehicle_schedule(cls, vehicle_status, vehicle_schedules):
        return Schedule.get_schedule_by_id(vehicle_schedules, vehicle_status.schedule_id)

    @classmethod
    def get_vehicle_schedule_events(cls, vehicle_schedules):
        return list(map(lambda x: x.event, vehicle_schedules))

    @classmethod
    def get_next_vehicle_schedule_index(cls, vehicle_status, vehicle_schedules):
        next_vehicle_schedule = Schedule.get_next_schedule_by_current_schedule_id(
            vehicle_schedules, vehicle_status.schedule_id)
        return vehicle_schedules.index(next_vehicle_schedule)

    @classmethod
    def get_next_vehicle_schedule_id(cls, vehicle_status, vehicle_schedules):
        return Schedule.get_next_schedule_by_current_schedule_id(vehicle_schedules, vehicle_status.schedule_id).id

    @classmethod
    def get_vehicle_config_key_and_value(cls, clients, target_roles):
        key = cls.get_vehicle_config_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Config.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_status_key_and_value(cls, clients, target_roles):
        key = cls.get_vehicle_status_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Status.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_schedules_key_and_value(cls, clients, target_roles):
        key = cls.get_vehicle_schedules_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "list":
            value = Schedule.new_schedules(value)
        return key, value

    @classmethod
    def set_vehicle_config(cls, clients, target_roles, vehicle_config, get_key=None):
        return clients["kvs"].set(
            cls.get_vehicle_config_key(target_roles),
            vehicle_config,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_status(cls, clients, target_roles, vehicle_status, get_key=None):
        return clients["kvs"].set(
            cls.get_vehicle_status_key(target_roles),
            vehicle_status,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_schedules(cls, clients, target_roles, vehicle_schedules, get_key=None):
        return clients["kvs"].set(
            cls.get_vehicle_schedules_key(target_roles),
            vehicle_schedules,
            get_key=get_key
        )

    @classmethod
    def update_and_set_vehicle_status(cls, clients, target_roles, vehicle_status, new_state, vehicle_schedules=None):
        vehicle_status.state = new_state
        vehicle_status.updated_at = Schedule.get_time()
        if vehicle_schedules is not None:
            vehicle_status.schedule_id = cls.get_next_vehicle_schedule_id(vehicle_status, vehicle_schedules)
        return cls.set_vehicle_status(clients, target_roles, vehicle_status)

    @classmethod
    def vehicle_activation_timeout(cls, vehicle_status):
        return cls.VEHICLE.ACTIVATION_REQUEST_TIMEOUT < Schedule.get_time() - vehicle_status.updated_at
