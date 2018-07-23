#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

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
    def get_dispatcher_config_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([Target.get_code(target_roles["dispatcher"]), "config"])

    @classmethod
    def get_dispatcher_state_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["dispatcher"]),
            "state"
        ])

    @classmethod
    def get_transportation_status_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["dispatcher"]), Target.get_code(target_roles["vehicle"]), "transportation"])

    @classmethod
    def get_user_status_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["dispatcher"]), Target.get_code(target_roles["user"]), "status"])

    @classmethod
    def get_user_schedules_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["dispatcher"]), Target.get_code(target_roles["user"]), "schedules"])

    @classmethod
    def get_vehicle_config_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["dispatcher"]), Target.get_code(target_roles["vehicle"]), "config"])

    @classmethod
    def get_vehicle_status_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["dispatcher"]), Target.get_code(target_roles["vehicle"]), "status"])

    @classmethod
    def get_vehicle_schedules_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["dispatcher"]), Target.get_code(target_roles["vehicle"]), "schedules"])

    @classmethod
    def get_dispatcher_config_key_and_value(cls, clients, target_roles):
        key = cls.get_dispatcher_config_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Config.new_data(**value)
        return key, value

    @classmethod
    def get_dispatcher_state_key_and_value(cls, clients, target_roles):
        key = cls.get_dispatcher_state_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.Status.new_data(**value)
        return key, value

    @classmethod
    def get_transportation_status_key_and_value(cls, clients, target_roles):
        key = cls.get_transportation_status_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.TransportationStatus.new_data(**value)
        return key, value

    # @classmethod
    # def get_user_status_key_and_value(cls, clients, target_roles):
    #     key = cls.get_user_status_key(target_roles)
    #     value = clients["kvs"].get(key)
    #     if value.__class__.__name__ == "dict":
    #         value = UserStatus.new_data(**value)
    #     return key, value

    @classmethod
    def get_user_schedules_key_and_value(cls, clients, target_roles):
        key = cls.get_user_schedules_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "list":
            value = Schedule.new_schedules(value)
        return key, value

    @classmethod
    def get_vehicle_config_key_and_value(cls, clients, target_roles):
        key = cls.get_vehicle_config_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.VehicleStructure.Config.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_status_key_and_value(cls, clients, target_roles):
        key = cls.get_vehicle_status_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.VehicleStructure.Status.new_data(**value)
        return key, value

    @classmethod
    def get_vehicle_schedules_key_and_value(cls, clients, target_roles):
        key = cls.get_vehicle_schedules_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "list":
            value = Schedules.new_data(value)
        return key, value

    @classmethod
    def get_activation_vehicle_schedules(cls, _clients, targets, current_time):
        return [
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.ACTIVATE,
                start_time=current_time,
                end_time=current_time + 100,
                route=None
            )
        ]

    @classmethod
    def get_transportation_vehicle_schedules(cls, _clients, targets, current_time):
        return [
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.START_MISSION,
                start_time=current_time,
                end_time=current_time + 10,
                route=None
            ),
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.END_MISSION,
                start_time=current_time,
                end_time=current_time,
                route=None
            )
        ]

    @classmethod
    def get_deactivation_vehicle_schedules(cls, _clients, targets, current_time):
        return [
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.DEACTIVATE,
                start_time=current_time,
                end_time=current_time + 100,
                route=None
            ),
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.END,
                start_time=current_time,
                end_time=current_time + 100,
                route=None
            )
        ]

    @classmethod
    def get_vehicle_schedules(
            cls, clients, target_roles, transportation_status, vehicle_status,
            vehicle_config, dispatcher_config):

        activation_vehicle_schedules = cls.get_activation_vehicle_schedules(
            clients, transportation_status.targets, cls.get_current_time())

        transportation_vehicle_schedules = cls.get_transportation_vehicle_schedules(
            clients, transportation_status.targets, activation_vehicle_schedules[-1].period.end)

        deactivation_vehicle_schedules = cls.get_deactivation_vehicle_schedules(
            clients, transportation_status.targets, transportation_vehicle_schedules[-1].period.end)

        return activation_vehicle_schedules + transportation_vehicle_schedules + deactivation_vehicle_schedules

    @classmethod
    def get_uuid(cls):
        return str(uuid())

    @classmethod
    def get_current_time(cls):
        return time()

    @classmethod
    def set_dispatcher_config(cls, clients, target_roles, dispatcher_config, get_key=None, timestamp_string=None):
        return clients["kvs"].set(
            cls.get_dispatcher_config_key(target_roles),
            dispatcher_config,
            get_key=get_key,
            timestamp_string=timestamp_string
        )

    @classmethod
    def set_dispatcher_state(cls, clients, target_roles, dispatcher_status, get_key=None):
        return clients["kvs"].set(
            cls.get_dispatcher_state_key(target_roles),
            dispatcher_status.state,
            get_key=get_key
        )

    @classmethod
    def set_transportation_status(
            cls, clients, target_roles, transportation_status, get_key=None):
        return clients["kvs"].set(
            cls.get_transportation_status_key(target_roles),
            transportation_status,
            get_key=get_key
        )

    @classmethod
    def set_user_status(cls, clients, target_roles, user_status, timestamp_string=None):
        return clients["kvs"].set(
            cls.get_user_status_key(target_roles),
            user_status,
            timestamp_string=timestamp_string
        )

    @classmethod
    def set_user_schedules(cls, clients, target_roles, user_schedules, get_key=None):
        return clients["kvs"].set(
            cls.get_user_schedules_key(target_roles),
            user_schedules,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_config(cls, clients, target_roles, vehicle_config, get_key=None):
        return clients["kvs"].set(
            cls.get_vehicle_config_key(target_roles),
            vehicle_config,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_status(cls, clients, target_roles, vehicle_status, timestamp_string=None):
        return clients["kvs"].set(
            cls.get_vehicle_status_key(target_roles),
            vehicle_status,
            timestamp_string=timestamp_string
        )

    @classmethod
    def set_vehicle_schedules(cls, clients, target_roles, vehicle_schedules, get_key=None):
        return clients["kvs"].set(
            cls.get_vehicle_schedules_key(target_roles),
            vehicle_schedules,
            get_key=get_key
        )

    @classmethod
    def update_and_set_transportation_status(
            cls, clients, target_roles, transportation_status, new_state, get_key):
        transportation_status.state = new_state
        transportation_status.updated_at = cls.get_current_time()
        return cls.set_transportation_status(clients, target_roles, transportation_status, get_key)

    @classmethod
    def delete_vehicle_config(cls, clients, target_roles):
        clients["kvs"].delete(
            cls.get_vehicle_config_key(target_roles)
        )
        return True

    @classmethod
    def delete_transportation_status(cls, clients, target_roles):
        clients["kvs"].delete(
            cls.get_transportation_status_key(target_roles)
        )
        return True
