#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Schedule
from ams.nodes.vehicle import CONST as VEHICLE
from ams.nodes.dispatcher import CONST, Structure, Helper, Publisher


class Condition(object):

    Helper = Helper

    @classmethod
    def vehicle_state_is_expected_state(cls, vehicle_state, expected):
        return vehicle_state == expected

    @classmethod
    def vehicle_config_existance(cls, vehicle_config):
        return vehicle_config is not None

    @classmethod
    def vehicle_api_key_is_avalable(cls, vehicle_config, dispatcher_config):
        return vehicle_config.activation in dispatcher_config.inactive_api_keys


class BeforeHook(object):

    Helper = Helper
    Publisher = Publisher

    @classmethod
    def set_vehicle_api_key_to_active_list(cls, clients, target_roles, vehicle_config, dispatcher_config):
        dispatcher_config.active_api_keys.append(vehicle_config.activation)
        dispatcher_config.inactive_api_keys.remove(vehicle_config.activation)
        return cls.Helper.set_dispatcher_config(clients, target_roles, dispatcher_config)


class AfterHook(object):

    Helper = Helper
    Publisher = Publisher

    @classmethod
    def get_vehicle_schedules(
            cls, clients, target_roles, transportation_status, vehicle_status, vehicle_config, dispatcher_config):
        return cls.Helper.get_vehicle_schedules(
            clients, target_roles, transportation_status, vehicle_status, vehicle_config, dispatcher_config)

    @classmethod
    def set_vehicle_schedules(cls, clients, target_roles, vehicle_schedules, vehicle_status_key):
        return cls.Helper.set_vehicle_schedules(clients, target_roles, vehicle_schedules, vehicle_status_key)

    @classmethod
    def publish_transportation_status(cls, clients, target_roles, transpotation_status):
        cls.Publisher.publish_transportation_status_message(clients, target_roles, transpotation_status)

    @classmethod
    def publish_vehicle_schedules(cls, clients, target_roles, vehicle_schedules):
        cls.Publisher.publish_schedules_message(clients, target_roles, vehicle_schedules)

    @classmethod
    def set_vehicle_api_key_to_inactive_list(cls, clients, target_roles, vehicle_config, dispatcher_config):
        if vehicle_config.activation not in dispatcher_config.active_api_keys:
            dispatcher_config.inactive_api_keys.append(vehicle_config.activation)
        if vehicle_config.activation in dispatcher_config.inactive_api_keys:
            dispatcher_config.active_api_keys.remove(vehicle_config.activation)
        return cls.Helper.set_dispatcher_config(clients, target_roles, dispatcher_config)

    @classmethod
    def delete_vehicle_config(cls, clients, target_roles):
        return cls.Helper.delete_vehicle_config(clients, target_roles)

    @classmethod
    def delete_transportation_status(cls, clients, target_roles):
        return cls.Helper.delete_transportation_status(clients, target_roles)


class Transition(object):

    VEHICLE = VEHICLE
    DISPATCHER = CONST

    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook

    @classmethod
    def start_processing_to_handshake(cls, clients, target_roles, transportation_status, vehicle_status_key):
        cls.Helper.update_and_set_transportation_status(
            clients, target_roles, transportation_status, cls.DISPATCHER.TRANSPORTATION.STATE.HANDSHAKE,
            vehicle_status_key)
        cls.AfterHook.publish_transportation_status(clients, target_roles, transportation_status)

        return True

    @classmethod
    def handshake_to_waiting_for_vehicle_state_end_processing(
            cls, clients, target_roles, transportation_status, vehicle_status,
            vehicle_config, dispatcher_config, vehicle_status_key):
        if cls.Condition.vehicle_state_is_expected_state(vehicle_status.state, cls.VEHICLE.STATE.INITIALIZED):
            if cls.Condition.vehicle_config_existance(vehicle_config):
                if cls.Condition.vehicle_api_key_is_avalable(vehicle_config, dispatcher_config):
                    if cls.BeforeHook.set_vehicle_api_key_to_active_list(
                            clients, target_roles, vehicle_config, dispatcher_config):

                        cls.Helper.update_and_set_transportation_status(
                            clients, target_roles, transportation_status,
                            cls.DISPATCHER.TRANSPORTATION.STATE.WAITING_FOR_VEHICLE_STATE_END_PROCESSING,
                            vehicle_status_key)

                        vehicle_schedules = cls.AfterHook.get_vehicle_schedules(
                            clients, target_roles, transportation_status, vehicle_status, vehicle_config,
                            dispatcher_config)
                        set_flag = cls.AfterHook.set_vehicle_schedules(
                            clients, target_roles, vehicle_schedules, vehicle_status_key)
                        if set_flag:
                            cls.AfterHook.publish_vehicle_schedules(clients, target_roles, vehicle_schedules)
                        return True
        return False

    @classmethod
    def waiting_for_vehicle_state_end_processing_to_end_processing(
            cls, clients, target_roles, transportation_status, vehicle_status,
            vehicle_status_key, vehicle_config, dispatcher_config):
        if cls.Condition.vehicle_state_is_expected_state(vehicle_status.state, cls.VEHICLE.STATE.END_PROCESSING):
            cls.Helper.update_and_set_transportation_status(
                clients, target_roles, transportation_status,
                cls.DISPATCHER.TRANSPORTATION.STATE.END_PROCESSING, vehicle_status_key)

            cls.AfterHook.set_vehicle_api_key_to_inactive_list(clients, target_roles, vehicle_config, dispatcher_config)
            cls.AfterHook.delete_vehicle_config(clients, target_roles)
            cls.AfterHook.delete_transportation_status(clients, target_roles)
            return True
        return False

    @classmethod
    def end_processing_to_start_processing(cls, clients, target_roles, transportation_status, vehicle_status_key):
        return cls.Helper.update_and_set_transportation_status(
            clients, target_roles, transportation_status, cls.DISPATCHER.TRANSPORTATION.STATE.START_PROCESSING,
            vehicle_status_key)


class StateMachine(object):

    DISPATCHER = CONST
    Structure = Structure
    Helper = Helper
    Transition = Transition

    VEHICLE = VEHICLE

    @classmethod
    def update_transportation_state(cls, clients, target_roles):

        transportation_status_key, transportation_status = cls.Helper.get_transportation_status_key_and_value(
            clients, target_roles)
        if transportation_status is None:
            transportation_status = cls.Structure.TransportationStatus.new_data(
                targets=[target_roles[cls.VEHICLE.ROLE_NAME]],
                state=cls.DISPATCHER.TRANSPORTATION.STATE.START_PROCESSING,
                updated_at=Schedule.get_time()
            )

        vehicle_status_key, vehicle_status = \
            cls.Helper.get_vehicle_status_key_and_value(clients, target_roles)

        dispatcher_config_key, dispatcher_config = cls.Helper.get_dispatcher_config_key_and_value(clients, target_roles)

        vehicle_config_key, vehicle_config = cls.Helper.get_vehicle_config_key_and_value(clients, target_roles)

        state = transportation_status.state
        if state == cls.DISPATCHER.TRANSPORTATION.STATE.START_PROCESSING:
            update_flag = cls.Transition.start_processing_to_handshake(
                clients, target_roles, transportation_status, vehicle_status_key)
        elif state == cls.DISPATCHER.TRANSPORTATION.STATE.HANDSHAKE:
            update_flag = cls.Transition.handshake_to_waiting_for_vehicle_state_end_processing(
                clients, target_roles, transportation_status, vehicle_status,
                vehicle_config, dispatcher_config, vehicle_status_key)
        elif state == cls.DISPATCHER.TRANSPORTATION.STATE.WAITING_FOR_VEHICLE_STATE_END_PROCESSING:
            update_flag = cls.Transition.waiting_for_vehicle_state_end_processing_to_end_processing(
                clients, target_roles, transportation_status, vehicle_status,
                vehicle_status_key, vehicle_config, dispatcher_config)
        elif state == cls.DISPATCHER.TRANSPORTATION.STATE.END_PROCESSING:
            update_flag = cls.Transition.end_processing_to_start_processing(
                clients, target_roles, transportation_status, vehicle_status_key)
        else:
            raise ValueError("Unknown Dispatcher State: {}".format(state))

        return update_flag
