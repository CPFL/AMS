#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Schedule
from ams.nodes.infra import CONST, Helper, Publisher


class Condition(object):

    CONST = CONST
    Helper = Helper

    @classmethod
    def pose_existance(cls, status):
        return status.pose is not None

    @classmethod
    def controller_assigned(cls, config):
        return config.target_controller is not None

    @classmethod
    def activation_timeout(cls, status):
        return Helper.activation_timeout(status)

    @classmethod
    def schedules_existance(cls, schedules):
        if schedules is None:
            return False
        return 0 < len(schedules)

    @classmethod
    def schedules_include_any_expected_events(cls, schedules, expected_events):
        schedule_events = cls.Helper.get_schedule_events(schedules)
        return 0 < len(list(set(expected_events) & set(schedule_events)))


class BeforeHook(object):

    CONST = CONST
    Helper = Helper
    Publisher = Publisher


class AfterHook(object):

    Helper = Helper
    Publisher = Publisher

    @classmethod
    def update_next_schedule_end_time_with_current_time_and_duration(cls, status, schedules):
        current_time = Helper.get_time()
        next_schedule_index = cls.Helper.get_next_schedule_index(status, schedules)
        duration = schedules[next_schedule_index].period.end - \
            schedules[next_schedule_index].period.start
        schedules[next_schedule_index].period.start = current_time
        schedules[next_schedule_index].period.end = current_time + duration

    @classmethod
    def publish_config_to_target_controller(cls, clients, target_roles, config):
        cls.Publisher.publish_config(clients, target_roles, config)

    @classmethod
    def publish_status(cls, clients, target_roles, status):
        cls.Publisher.publish_status(clients, target_roles, status)


class Transition(object):

    CONST = CONST
    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook

    @classmethod
    def start_processing_to_initialized(cls, clients, target_roles, status, config):
        if all([
            cls.Condition.pose_existance(status),
            cls.Condition.controller_assigned(config),
        ]):
            cls.Helper.update_and_set_status(clients, target_roles, status, cls.CONST.STATE.INITIALIZED)
            cls.AfterHook.publish_config_to_target_controller(clients, target_roles, config)
            return True

        return False

    @classmethod
    def initialized_to_active(cls, clients, target_roles, status, schedules, schedules_key):
        if cls.Condition.schedules_existance(schedules):
            if cls.Condition.schedules_include_any_expected_events(schedules, cls.CONST.MISSION_EVENTS):
                return cls.Helper.update_and_set_status(
                    clients, target_roles, status, cls.CONST.STATE.ACTIVE, schedules_key)
        return False

    @classmethod
    def active_to_mission_started(cls, clients, target_roles, status, schedules, schedules_key):
        cls.Helper.update_and_set_status(
            clients, target_roles, status, cls.CONST.STATE.MISSION_STARTED, schedules, schedules_key)
        cls.AfterHook.update_next_schedule_end_time_with_current_time_and_duration(status, schedules)
        return True

    @classmethod
    def mission_started_to_mission_ended(cls, clients, target_roles, status, schedules, schedules_key):
        if cls.Condition.activation_timeout(status):
            return cls.Helper.update_and_set_status(
                clients, target_roles, status, cls.CONST.STATE.MISSION_ENDED, schedules, schedules_key)
        return False

    @classmethod
    def mission_ended_to_inactive(cls, clients, target_roles, status, schedules, schedules_key):
        return cls.Helper.update_and_set_status(
            clients, target_roles, status, cls.CONST.STATE.INACTIVE, schedules, schedules_key)

    @classmethod
    def to_end_processing(cls, clients, target_roles, status):
        return cls.Helper.update_and_set_status(clients, target_roles, status, cls.CONST.STATE.END_PROCESSING)


class EventHandler(object):

    CONST = CONST
    Transition = Transition

    @classmethod
    def start_mission(cls, clients, target_roles, status, schedules, schedules_key):
        state = status.state
        if state == cls.CONST.STATE.ACTIVE:
            return cls.Transition.active_to_mission_started(
                clients, target_roles, status, schedules, schedules_key)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))

    @classmethod
    def end_mission(cls, clients, target_roles, status, schedules, schedules_key):
        return cls.Transition.mission_started_to_mission_ended(
            clients, target_roles, status, schedules, schedules_key)

    @classmethod
    def deactivate(cls, clients, target_roles, status, schedules, schedules_key):
        state = status.state
        if state == cls.CONST.STATE.MISSION_ENDED:
            return cls.Transition.mission_ended_to_inactive(
                clients, target_roles, status, schedules, schedules_key)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))

    @classmethod
    def end(cls, clients, target_roles, status):
        state = status.state
        if state == cls.CONST.STATE.INACTIVE:
            return cls.Transition.to_end_processing(clients, target_roles, status)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))


class StateMachine(object):

    CONST = CONST
    Helper = Helper
    EventHandler = EventHandler

    @classmethod
    def update_state(cls, clients, target_roles):
        config_key, config = cls.Helper.get_config_key_and_value(clients, target_roles)
        status_key, status = cls.Helper.get_status_key_and_value(clients, target_roles)
        schedules_key, schedules = cls.Helper.get_schedules_key_and_value(clients, target_roles)

        state = status.state

        if state in [
                cls.CONST.STATE.START_PROCESSING, cls.CONST.STATE.INITIALIZED,
                cls.CONST.STATE.INACTIVE, cls.CONST.STATE.END_PROCESSING
        ]:
            update_flag = cls.update_state_without_schedules(
                clients, target_roles, status, config, schedules, schedules_key)
        else:
            update_flag = cls.update_state_with_schedules(
                clients, target_roles, status, schedules, schedules_key)

        return update_flag

    @classmethod
    def update_state_without_schedules(
            cls, clients, target_roles, status, config, schedules, schedules_key):
        state = status.state
        if state == cls.CONST.STATE.START_PROCESSING:
            update_flag = cls.EventHandler.Transition.start_processing_to_initialized(
                clients, target_roles, status, config)
        elif state == cls.CONST.STATE.INITIALIZED:
            update_flag = cls.EventHandler.Transition.initialized_to_active(
                clients, target_roles, status, schedules, schedules_key)
        elif state == cls.CONST.STATE.INACTIVE:
            update_flag = cls.EventHandler.Transition.to_end_processing(clients, target_roles, status)
        elif state == cls.CONST.STATE.END_PROCESSING:
            update_flag = False
        else:
            raise ValueError("Transition from {} without event is undefined.".format(state))

        return update_flag

    @classmethod
    def update_state_with_schedules(
            cls, clients, target_roles, status, schedules, schedules_key):
        update_flag = False

        if status.schedule_id is None:
            status.schedule_id = schedules[0].id

        next_schedule = Schedule.get_next_schedule_by_current_schedule_id(schedules, status.schedule_id)

        if next_schedule is not None:
            event = next_schedule.event
            if event == cls.CONST.EVENT.START_MISSION:
                update_flag = cls.EventHandler.start_mission(
                    clients, target_roles, status, schedules, schedules_key)
            elif event == cls.CONST.EVENT.END_MISSION:
                update_flag = cls.EventHandler.end_mission(
                    clients, target_roles, status, schedules, schedules_key)
            elif event == cls.CONST.EVENT.DEACTIVATE:
                update_flag = cls.EventHandler.deactivate(
                    clients, target_roles, status, schedules, schedules_key)
            else:
                raise ValueError("Unknown Event {}.".format(event))

        return update_flag