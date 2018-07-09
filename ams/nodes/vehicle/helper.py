#!/usr/bin/env python
# coding: utf-8

from time import time

from ams.structures import KVS_CLIENT
from ams.helpers import Target, Schedule
from ams.nodes.vehicle import CONST, Structure


class Helper(object):

    VEHICLE = CONST
    VehicleStructure = Structure

    @classmethod
    def get_vehicle_config_key(cls, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([Target.get_code(target_vehicle), "config"])

    @classmethod
    def get_vehicle_status_key(cls, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([Target.get_code(target_vehicle), "status"])

    @classmethod
    def get_vehicle_schedules_key(cls, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([Target.get_code(target_vehicle), "schedules"])

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
    def get_vehicle_config_key_and_value(cls, kvs_client, target_vehicle):
        key = cls.get_vehicle_config_key(target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = cls.VehicleStructure.Config.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_status_key_and_value(cls, kvs_client, target_vehicle):
        key = cls.get_vehicle_status_key(target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = cls.VehicleStructure.Status.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_schedules_key_and_value(cls, kvs_client, target_vehicle):
        key = cls.get_vehicle_schedules_key(target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "list":
            value = Schedule.new_schedules(value)
        return key, value

    @classmethod
    def get_current_time(cls):
        return time()

    @classmethod
    def set_vehicle_config(cls, kvs_client, target_vehicle, vehicle_config, get_key=None):
        return kvs_client.set(
            cls.get_vehicle_config_key(target_vehicle),
            vehicle_config,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_status(cls, kvs_client, target_vehicle, vehicle_status, get_key=None):
        return kvs_client.set(
            cls.get_vehicle_status_key(target_vehicle),
            vehicle_status,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_schedules(cls, kvs_client, target_vehicle, vehicle_schedules, get_key=None):
        return kvs_client.set(
            cls.get_vehicle_schedules_key(target_vehicle),
            vehicle_schedules,
            get_key=get_key
        )

    @classmethod
    def update_and_set_vehicle_status(cls, kvs_client, target_vehicle, vehicle_status, new_state, vehicle_schedules=None):
        vehicle_status.state = new_state
        vehicle_status.updated_at = cls.get_current_time()
        if vehicle_schedules is not None:
            vehicle_status.schedule_id = cls.get_next_vehicle_schedule_id(vehicle_status, vehicle_schedules)
        return cls.set_vehicle_status(kvs_client, target_vehicle, vehicle_status)

    @classmethod
    def vehicle_activation_timeout(cls, vehicle_status):
        return cls.VEHICLE.ACTIVATION_REQUEST_TIMEOUT < cls.get_current_time() - vehicle_status.updated_at
