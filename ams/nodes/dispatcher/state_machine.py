#!/usr/bin/env python
# coding: utf-8

from time import time

from ams.nodes.vehicle import CONST as VEHICLE
from ams.nodes.dispatcher import CONST, Structure, Helper, Publisher


class Condition(object):

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

    @classmethod
    def set_vehicle_api_key_to_active_list(cls, vehicle_config, dispatcher_config, kvs_client, target_dispatcher):
        dispatcher_config.active_api_keys.append(vehicle_config.activation)
        dispatcher_config.inactive_api_keys.remove(vehicle_config.activation)
        return cls.Helper.set_dispatcher_config(kvs_client, target_dispatcher, dispatcher_config)


class AfterHook(object):

    Helper = Helper
    Publisher = Publisher

    @classmethod
    def get_vehicle_schedules(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
            vehicle_config, dispatcher_config, maps_client):
        return cls.Helper.get_vehicle_schedules(
            kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
            vehicle_config, dispatcher_config, maps_client)

    @classmethod
    def set_vehicle_schedules(
            cls, kvs_client, target_dispatcher, target_vehicle, vehicle_schedules, vehicle_status_key):
        return cls.Helper.set_vehicle_schedules(
            kvs_client, target_dispatcher, target_vehicle, vehicle_schedules, vehicle_status_key)

    @classmethod
    def publish_transportation_status(cls, mqtt_client, target_dispatcher, target_vehicle, transpotation_status):
        cls.Publisher.publish_transportation_status_message(
            mqtt_client, target_dispatcher, target_vehicle, transpotation_status)

    @classmethod
    def publish_vehicle_schedules(cls, mqtt_client, target_dispatcher, target_vehicle, vehicle_schedules):
        cls.Publisher.publish_schedules_message(mqtt_client, target_dispatcher, target_vehicle, vehicle_schedules)

    @classmethod
    def set_vehicle_api_key_to_inactive_list(cls, vehicle_config, dispatcher_config, kvs_client, target_dispatcher):
        dispatcher_config.inactive_api_keys.append(vehicle_config.activation)
        dispatcher_config.active_api_keys.remove(vehicle_config.activation)
        return cls.Helper.set_dispatcher_config(kvs_client, target_dispatcher, dispatcher_config)

    @classmethod
    def delete_vehicle_config(cls, kvs_client, target_dispatcher, target_vehicle):
        return cls.Helper.delete_vehicle_config(kvs_client, target_dispatcher, target_vehicle)

    @classmethod
    def delete_transportation_status(cls, kvs_client, target_dispatcher, target_vehicle):
        return cls.Helper.delete_transportation_status(kvs_client, target_dispatcher, target_vehicle)


class Transition(object):

    VEHICLE = VEHICLE
    DISPATCHER = CONST

    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook

    @classmethod
    def start_processing_to_handshake(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status_key, mqtt_client):
        cls.Helper.update_and_set_transportation_status(
            kvs_client, target_dispatcher, target_vehicle, transportation_status,
            cls.DISPATCHER.TRANSPORTATION.STATE.HANDSHAKE, vehicle_status_key)
        cls.AfterHook.publish_transportation_status(
            mqtt_client, target_dispatcher, target_vehicle, transportation_status)

        return True

    @classmethod
    def handshake_to_waiting_for_vehicle_state_end_processing(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
            vehicle_config, dispatcher_config, vehicle_status_key, mqtt_client, maps_client):
        print("vehicle_state_is_expected_state", vehicle_status.state, cls.VEHICLE.STATE.INITIALIZED)
        if cls.Condition.vehicle_state_is_expected_state(vehicle_status.state, cls.VEHICLE.STATE.INITIALIZED):
            print("vehicle_config_existance", vehicle_config, dispatcher_config)
            if cls.Condition.vehicle_config_existance(vehicle_config):
                print("vehicle_api_key_is_avalable", vehicle_config, dispatcher_config)
                if cls.Condition.vehicle_api_key_is_avalable(vehicle_config, dispatcher_config):
                    if cls.BeforeHook.set_vehicle_api_key_to_active_list(
                            vehicle_config, dispatcher_config, kvs_client, target_dispatcher):

                        cls.Helper.update_and_set_transportation_status(
                            kvs_client, target_dispatcher, target_vehicle, transportation_status,
                            cls.DISPATCHER.TRANSPORTATION.STATE.WAITING_FOR_VEHICLE_STATE_END_PROCESSING,
                            vehicle_status_key)

                        vehicle_schedules = cls.AfterHook.get_vehicle_schedules(
                            kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
                            vehicle_config, dispatcher_config, maps_client)
                        set_flag = cls.AfterHook.set_vehicle_schedules(
                            kvs_client, target_dispatcher, target_vehicle, vehicle_schedules, vehicle_status_key)
                        if set_flag:
                            cls.AfterHook.publish_vehicle_schedules(
                                mqtt_client, target_dispatcher, target_vehicle, vehicle_schedules)
                        return True
        return False

    @classmethod
    def waiting_for_vehicle_state_end_processing_to_end_processing(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
            vehicle_status_key, vehicle_config, dispatcher_config):
        if cls.Condition.vehicle_state_is_expected_state(vehicle_status.state, cls.VEHICLE.STATE.END_PROCESSING):
            cls.Helper.update_and_set_transportation_status(
                kvs_client, target_dispatcher, target_vehicle, transportation_status,
                cls.DISPATCHER.TRANSPORTATION.STATE.END_PROCESSING, vehicle_status_key)

            cls.AfterHook.set_vehicle_api_key_to_inactive_list(
                vehicle_config, dispatcher_config, kvs_client, target_dispatcher)
            cls.AfterHook.delete_vehicle_config(kvs_client, target_dispatcher, target_vehicle)
            cls.AfterHook.delete_transportation_status(kvs_client, target_dispatcher, target_vehicle)
            return True
        return False

    @classmethod
    def end_processing_to_start_processing(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status_key):
        return cls.Helper.update_and_set_transportation_status(
            kvs_client, target_dispatcher, target_vehicle, transportation_status,
            cls.DISPATCHER.TRANSPORTATION.STATE.START_PROCESSING, vehicle_status_key)


class StateMachine(object):

    DISPATCHER = CONST

    Helper = Helper
    Transition = Transition

    @classmethod
    def update_transportation_state(
            cls, target_dispatcher, target_vehicle, _target_user, kvs_client, mqtt_client=None, maps_client=None):

        transportation_status_key, transportation_status = cls.Helper.get_transportation_status_key_and_value(
            kvs_client, target_dispatcher, target_vehicle)
        if transportation_status is None:
            transportation_status = Structure.TransportationStatus.new_data(
                targets=[target_vehicle],
                state=cls.DISPATCHER.TRANSPORTATION.STATE.START_PROCESSING,
                updated_at=cls.Helper.get_current_time()
            )

        vehicle_status_key, vehicle_status = \
            cls.Helper.get_vehicle_status_key_and_value(kvs_client, target_dispatcher, target_vehicle)

        print("state, vehicle_state, vehicle_state.schedule_id",
              transportation_status.state, vehicle_status.state, vehicle_status.schedule_id)

        dispatcher_config_key, dispatcher_config = cls.Helper.get_dispatcher_config_key_and_value(
            kvs_client, target_dispatcher)

        vehicle_config_key, vehicle_config = \
            cls.Helper.get_vehicle_config_key_and_value(kvs_client, target_dispatcher, target_vehicle)

        state = transportation_status.state
        if state == cls.DISPATCHER.TRANSPORTATION.STATE.START_PROCESSING:
            update_flag = cls.Transition.start_processing_to_handshake(
                kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status_key, mqtt_client)
        elif state == cls.DISPATCHER.TRANSPORTATION.STATE.HANDSHAKE:
            update_flag = cls.Transition.handshake_to_waiting_for_vehicle_state_end_processing(
                kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
                vehicle_config, dispatcher_config, vehicle_status_key, mqtt_client, maps_client)
        elif state == cls.DISPATCHER.TRANSPORTATION.STATE.WAITING_FOR_VEHICLE_STATE_END_PROCESSING:
            update_flag = cls.Transition.waiting_for_vehicle_state_end_processing_to_end_processing(
                kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status,
                vehicle_status_key, vehicle_config, dispatcher_config)
        elif state == cls.DISPATCHER.TRANSPORTATION.STATE.END_PROCESSING:
            update_flag = cls.Transition.end_processing_to_start_processing(
                kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status_key)
        else:
            raise ValueError("Unknown Dispatcher State: {}".format(state))

        return update_flag
