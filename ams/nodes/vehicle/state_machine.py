#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Schedule
from ams.nodes.vehicle import CONST, Helper, Publisher


class Condition(object):

    VEHICLE = CONST
    Helper = Helper

    @classmethod
    def vehicle_located(cls, vehicle_status):
        return vehicle_status.location is not None

    @classmethod
    def dispatcher_assigned(cls, vehicle_config):
        return vehicle_config.target_dispatcher is not None

    @classmethod
    def vehicle_activation_timeout(cls, vehicle_status):
        return Helper.vehicle_activation_timeout(vehicle_status)

    @classmethod
    def vehicle_schedules_existance(cls, vehicle_schedules):
        if vehicle_schedules is None:
            return False
        return 0 < len(vehicle_schedules)

    @classmethod
    def vehicle_schedules_include_any_expected_events(cls, vehicle_schedules, expected_events):
        schedule_events = cls.Helper.get_vehicle_schedule_events(vehicle_schedules)
        return 0 < len(list(set(expected_events) & set(schedule_events)))


class BeforeHook(object):

    VEHICLE = CONST
    Helper = Helper
    Publisher = Publisher


class AfterHook(object):

    Helper = Helper
    Publisher = Publisher

    @classmethod
    def update_next_vehicle_schedule_end_time_with_current_time_and_duration(cls, vehicle_status, vehicle_schedules):
        current_time = Helper.get_current_time()
        next_vehicle_schedule_index = cls.Helper.get_next_vehicle_schedule_index(vehicle_status, vehicle_schedules)
        duration = vehicle_schedules[next_vehicle_schedule_index].period.end - \
            vehicle_schedules[next_vehicle_schedule_index].period.start
        vehicle_schedules[next_vehicle_schedule_index].period.start = current_time
        vehicle_schedules[next_vehicle_schedule_index].period.end = current_time + duration

    @classmethod
    def publish_vehicle_config_to_target_dispatcher(cls, clients, target_roles, vehicle_config):
        cls.Publisher.publish_vehicle_config(clients, target_roles, vehicle_config)

    @classmethod
    def publish_vehicle_status(cls, clients, target_roles, vehicle_status):
        cls.Publisher.publish_vehicle_status(clients, target_roles, vehicle_status)


class Transition(object):

    VEHICLE = CONST

    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook

    @classmethod
    def start_processing_to_initialized(cls, clients, target_roles, vehicle_status, vehicle_config):
        if all([
            cls.Condition.vehicle_located(vehicle_status),
            cls.Condition.dispatcher_assigned(vehicle_config),
        ]):
            cls.Helper.update_and_set_vehicle_status(clients, target_roles, vehicle_status, cls.VEHICLE.STATE.INITIALIZED)
            cls.AfterHook.publish_vehicle_config_to_target_dispatcher(clients, target_roles, vehicle_config)
            return True

        return False

    @classmethod
    def initialized_to_active(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        if cls.Condition.vehicle_schedules_existance(vehicle_schedules):
            if cls.Condition.vehicle_schedules_include_any_expected_events(
                    vehicle_schedules, cls.VEHICLE.MISSION_EVENTS):
                return cls.Helper.update_and_set_vehicle_status(
                    clients, target_roles, vehicle_status, cls.VEHICLE.STATE.ACTIVE)
        return False

    @classmethod
    def active_to_mission_started(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        cls.Helper.update_and_set_vehicle_status(
            clients, target_roles, vehicle_status, cls.VEHICLE.STATE.MISSION_STARTED, vehicle_schedules)
        cls.AfterHook.update_next_vehicle_schedule_end_time_with_current_time_and_duration(
            vehicle_status, vehicle_schedules)
        return True

    @classmethod
    def mission_started_to_mission_ended(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        if cls.Condition.vehicle_activation_timeout(vehicle_status):
            return cls.Helper.update_and_set_vehicle_status(
                clients, target_roles, vehicle_status, cls.VEHICLE.STATE.MISSION_ENDED, vehicle_schedules)
        return False

    @classmethod
    def mission_ended_to_inactive(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        return cls.Helper.update_and_set_vehicle_status(
            clients, target_roles, vehicle_status, cls.VEHICLE.STATE.INACTIVE, vehicle_schedules)

    @classmethod
    def to_end_processing(cls, clients, target_roles, vehicle_status):
        return cls.Helper.update_and_set_vehicle_status(
            clients, target_roles, vehicle_status, cls.VEHICLE.STATE.END_PROCESSING)


class EventHandler(object):

    VEHICLE = CONST

    Transition = Transition

    @classmethod
    def start_mission(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.ACTIVE:
            return cls.Transition.active_to_mission_started(
                clients, target_roles, vehicle_status, vehicle_schedules)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))

    @classmethod
    def end_mission(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        return cls.Transition.mission_started_to_mission_ended(
            clients, target_roles, vehicle_status, vehicle_schedules)

    @classmethod
    def deactivate(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.MISSION_ENDED:
            return cls.Transition.mission_ended_to_inactive(
                clients, target_roles, vehicle_status, vehicle_schedules)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))

    @classmethod
    def end(cls, clients, target_roles, vehicle_status):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.INACTIVE:
            return cls.Transition.to_end_processing(clients, target_roles, vehicle_status)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))


class StateMachine(object):

    VEHICLE = CONST

    Helper = Helper
    EventHandler = EventHandler

    @classmethod
    def update_vehicle_state(cls, clients, target_roles):
        vehicle_config_key, vehicle_config = cls.Helper.get_vehicle_config_key_and_value(clients, target_roles)
        vehicle_status_key, vehicle_status = cls.Helper.get_vehicle_status_key_and_value(clients, target_roles)
        vehicle_schedules_key, vehicle_schedules = cls.Helper.get_vehicle_schedules_key_and_value(clients, target_roles)

        state = vehicle_status.state

        if state in [
                cls.VEHICLE.STATE.START_PROCESSING, cls.VEHICLE.STATE.INITIALIZED,
                cls.VEHICLE.STATE.INACTIVE, cls.VEHICLE.STATE.END_PROCESSING
        ]:
            update_flag = cls.update_vehicle_state_without_schedules(
                clients, target_roles, vehicle_status, vehicle_config, vehicle_schedules)
        else:
            update_flag = cls.update_vehicle_state_with_schedules(
                clients, target_roles, vehicle_status, vehicle_schedules)

        return update_flag

    @classmethod
    def update_vehicle_state_without_schedules(
            cls, clients, target_roles, vehicle_status, vehicle_config, vehicle_schedules):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.START_PROCESSING:
            update_flag = cls.EventHandler.Transition.start_processing_to_initialized(
                clients, target_roles, vehicle_status, vehicle_config)
        elif state == cls.VEHICLE.STATE.INITIALIZED:
            update_flag = cls.EventHandler.Transition.initialized_to_active(
                clients, target_roles, vehicle_status, vehicle_schedules)
        elif state == cls.VEHICLE.STATE.INACTIVE:
            update_flag = cls.EventHandler.Transition.to_end_processing(clients, target_roles, vehicle_status)
        elif state == cls.VEHICLE.STATE.END_PROCESSING:
            update_flag = False
        else:
            raise ValueError("Transition from {} without event is undefined.".format(state))

        return update_flag

    @classmethod
    def update_vehicle_state_with_schedules(
            cls, clients, target_roles, vehicle_status, vehicle_schedules):
        update_flag = False

        if vehicle_status.schedule_id is None:
            vehicle_status.schedule_id = vehicle_schedules[0].id

        next_schedule = Schedule.get_next_schedule_by_current_schedule_id(vehicle_schedules, vehicle_status.schedule_id)

        if next_schedule is not None:
            event = next_schedule.event
            if event == cls.VEHICLE.EVENT.START_MISSION:
                update_flag = cls.EventHandler.start_mission(
                    clients, target_roles, vehicle_status, vehicle_schedules)
            elif event == cls.VEHICLE.EVENT.END_MISSION:
                update_flag = cls.EventHandler.end_mission(
                    clients, target_roles, vehicle_status, vehicle_schedules)
            elif event == cls.VEHICLE.EVENT.DEACTIVATE:
                update_flag = cls.EventHandler.deactivate(
                    clients, target_roles, vehicle_status, vehicle_schedules)
            else:
                raise ValueError("Unknown Event {}.".format(event))

        return update_flag
