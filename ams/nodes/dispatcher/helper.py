#!/usr/bin/env python
# coding: utf-8

from time import time

from ams.structures import KVS_CLIENT, Schedules
from ams.helpers import Target, Schedule
from ams.nodes.vehicle import CONST as VEHICLE
from ams.nodes.vehicle import Structure as VehicleStructure
from ams.nodes.dispatcher import Structure


class Helper(object):

    VEHICLE = VEHICLE
    VehicleStructure = VehicleStructure
    Structure = Structure

    @classmethod
    def get_dispatcher_config_key(cls, target_dispatcher):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([Target.get_code(target_dispatcher), "config"])

    @classmethod
    def get_dispatcher_state_key(cls, target_dispatcher):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_dispatcher),
            "state"
        ])

    @classmethod
    def get_transportation_status_key(cls, target_dispatcher, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_dispatcher), Target.get_code(target_vehicle), "transportation"])

    @classmethod
    def get_user_status_key(cls, target_dispatcher, target_user):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_dispatcher), Target.get_code(target_user), "status"])

    @classmethod
    def get_user_schedules_key(cls, target_dispatcher, target_user):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_dispatcher), Target.get_code(target_user), "schedules"])

    @classmethod
    def get_vehicle_config_key(cls, target_dispatcher, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_dispatcher), Target.get_code(target_vehicle), "config"])

    @classmethod
    def get_vehicle_status_key(cls, target_dispatcher, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_dispatcher), Target.get_code(target_vehicle), "status"])

    @classmethod
    def get_vehicle_schedules_key(cls, target_dispatcher, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_dispatcher), Target.get_code(target_vehicle), "schedules"])

    @classmethod
    def get_dispatcher_config_key_and_value(cls, kvs_client, target_dispatcher):
        key = cls.get_dispatcher_config_key(target_dispatcher)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Config.new_data(**value)
        return key, value

    @classmethod
    def get_dispatcher_state_key_and_value(cls, kvs_client, target_dispatcher):
        key = cls.get_dispatcher_state_key(target_dispatcher)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Status.new_data(**value)
        return key, value

    @classmethod
    def get_transportation_status_key_and_value(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_transportation_status_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.TransportationStatus.new_data(**value)
        return key, value

    # @classmethod
    # def get_user_status_key_and_value(cls, kvs_client, target_dispatcher, target_user):
    #     key = cls.get_user_status_key(target_dispatcher, target_user)
    #     value = kvs_client.get(key)
    #     if value.__class__.__name__ == "dict":
    #         value = UserStatus.new_data(**value)
    #     return key, value

    @classmethod
    def get_user_schedules_key_and_value(cls, kvs_client, target_dispatcher, target_user):
        key = cls.get_user_schedules_key(target_dispatcher, target_user)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "list":
            value = Schedule.new_schedules(value)
        return key, value

    @classmethod
    def get_vehicle_config_key_and_value(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_vehicle_config_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = cls.VehicleStructure.Config.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_status_key_and_value(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_vehicle_status_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = cls.VehicleStructure.Status.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_schedules_key_and_value(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_vehicle_schedules_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "list":
            value = Schedules.new_data(value)
        return key, value

    @classmethod
    def get_activation_vehicle_schedules(cls, transportation_targets, current_time):
        return [
            Schedule.new_schedule(
                targets=transportation_targets,
                event=cls.VEHICLE.EVENT.ACTIVATE,
                start_time=current_time,
                end_time=current_time + 100,
                route=None
            )
        ]

    @classmethod
    def get_transportation_vehicle_schedules(cls, transportation_targets, current_time):
        return [
            Schedule.new_schedule(
                targets=transportation_targets,
                event=cls.VEHICLE.EVENT.START_MISSION,
                start_time=current_time,
                end_time=current_time + 10,
                route=None
            ),
            Schedule.new_schedule(
                targets=transportation_targets,
                event=cls.VEHICLE.EVENT.END_MISSION,
                start_time=current_time,
                end_time=current_time,
                route=None
            )
        ]

    @classmethod
    def get_deactivation_vehicle_schedules(cls, transportation_targets, current_time):
        return [
            Schedule.new_schedule(
                targets=transportation_targets,
                event=cls.VEHICLE.EVENT.DEACTIVATE,
                start_time=current_time,
                end_time=current_time + 100,
                route=None
            ),
            Schedule.new_schedule(
                targets=transportation_targets,
                event=cls.VEHICLE.EVENT.END,
                start_time=current_time,
                end_time=current_time + 100,
                route=None
            )
        ]

    @classmethod
    def get_vehicle_schedules(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
            vehicle_config, dispatcher_config, maps_client):
        current_time = time()
        activation_vehicle_schedules = cls.get_activation_vehicle_schedules(transportation_status.targets, current_time)
        transportation_vehicle_schedules = cls.get_transportation_vehicle_schedules(
            transportation_status.targets, activation_vehicle_schedules[-1].period.end)
        deactivation_vehicle_schedules = cls.get_deactivation_vehicle_schedules(
            transportation_status.targets, transportation_vehicle_schedules[-1].period.end)
        return activation_vehicle_schedules + transportation_vehicle_schedules + deactivation_vehicle_schedules

    @classmethod
    def get_current_time(cls):
        return time()

    @classmethod
    def set_dispatcher_config(cls, kvs_client, target_dispatcher, dispatcher_config, get_key=None):
        return kvs_client.set(
            cls.get_dispatcher_config_key(target_dispatcher),
            dispatcher_config,
            get_key=get_key
        )

    @classmethod
    def set_dispatcher_state(cls, kvs_client, target_dispatcher, dispatcher_status, get_key=None):
        return kvs_client.set(
            cls.get_dispatcher_state_key(target_dispatcher),
            dispatcher_status.state,
            get_key=get_key
        )

    @classmethod
    def set_transportation_status(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, get_key=None):
        return kvs_client.set(
            cls.get_transportation_status_key(target_dispatcher, target_vehicle),
            transportation_status,
            get_key=get_key
        )

    @classmethod
    def set_user_status(cls, kvs_client, target_dispatcher, target_user, user_status, timestamp_string=None):
        return kvs_client.set(
            cls.get_user_status_key(target_dispatcher, target_user),
            user_status,
            timestamp_string=timestamp_string
        )

    @classmethod
    def set_user_schedules(cls, kvs_client, target_dispatcher, target_user, user_schedules, get_key=None):
        return kvs_client.set(
            cls.get_user_schedules_key(target_dispatcher, target_user),
            user_schedules,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_config(cls, kvs_client, target_dispatcher, target_vehicle, vehicle_config, get_key=None):
        return kvs_client.set(
            cls.get_vehicle_config_key(target_dispatcher, target_vehicle),
            vehicle_config,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_status(cls, kvs_client, target_dispatcher, target_vehicle, vehicle_status, timestamp_string=None):
        return kvs_client.set(
            cls.get_vehicle_status_key(target_dispatcher, target_vehicle),
            vehicle_status,
            timestamp_string=timestamp_string
        )

    @classmethod
    def set_vehicle_schedules(cls, kvs_client, target_dispatcher, target_vehicle, vehicle_schedules, get_key=None):
        return kvs_client.set(
            cls.get_vehicle_schedules_key(target_dispatcher, target_vehicle),
            vehicle_schedules,
            get_key=get_key
        )

    @classmethod
    def update_and_set_transportation_status(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, new_state, get_key):
        transportation_status.state = new_state
        transportation_status.updated_at = cls.get_current_time()
        return cls.set_transportation_status(
            kvs_client, target_dispatcher, target_vehicle, transportation_status, get_key)

    @classmethod
    def delete_vehicle_config(cls, kvs_client, target_dispatcher, target_vehicle):
        kvs_client.delete(
            cls.get_vehicle_config_key(target_dispatcher, target_vehicle)
        )
        return True

    @classmethod
    def delete_transportation_status(cls, kvs_client, target_dispatcher, target_vehicle):
        kvs_client.delete(
            cls.get_transportation_status_key(target_dispatcher, target_vehicle)
        )
        return True
