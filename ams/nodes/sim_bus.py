#!/usr/bin/env python
# coding: utf-8

from time import time
from copy import deepcopy

from transitions import Machine

from ams import logger, Topic, Target
from ams.nodes import SimCar
from ams.messages import UserStatus
from ams.structures import SIM_BUS, SIM_BUS_USER, USER


class SimBus(SimCar):

    CONST = SIM_BUS

    def __init__(
            self, _id, name, waypoint, arrow, route, intersection, dt=1.0):
        super().__init__(_id, name, waypoint, arrow, route, intersection, dt=dt)

        self.state_machine = self.get_state_machine(SIM_BUS.STATE.STAND_BY)

        self.user_statuses = self.manager.dict()
        self.user_statuses_lock = self.manager.Lock()

        self.__topicSubUserStatus = Topic()
        self.__topicSubUserStatus.set_targets(Target.new_target(None, SIM_BUS_USER.NODE_NAME), None)
        self.__topicSubUserStatus.set_categories(USER.TOPIC.CATEGORIES.STATUS)
        self.__topicSubUserStatus.set_message(UserStatus)
        self.set_subscriber(self.__topicSubUserStatus, self.update_user_status)

    def update_user_status(self, _client, _userdata, topic, payload):
        user_id = self.__topicSubUserStatus.get_from_id(topic)
        user_status = self.__topicSubUserStatus.unserialize(payload)

        self.user_statuses_lock.acquire()
        # if user_id in self.user_statuses or user_status.state == USER.STATE.LOG_IN:
        if user_status.state in [USER.STATE.LOG_OUT]:
            if user_id in self.user_statuses:
                self.user_statuses.pop(user_id)
        else:
            self.user_statuses[user_id] = user_status
        self.user_statuses_lock.release()

    def get_state_machine(self, initial_state):
        machine = Machine(
            states=list(SIM_BUS.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": SIM_BUS.TRIGGER.MOVE,
                "source": SIM_BUS.STATE.STAND_BY, "dest": SIM_BUS.STATE.MOVE_TO_CIRCULAR_ROUTE,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
            {
                "trigger": SIM_BUS.TRIGGER.MOVE,
                "source": SIM_BUS.STATE.MOVE_TO_CIRCULAR_ROUTE, "dest": SIM_BUS.STATE.MOVE_TO_SELECT_POINT,
                "conditions": [self.condition_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_BUS.TRIGGER.MOVE,
                "source": SIM_BUS.STATE.MOVE_TO_SELECT_POINT, "dest": SIM_BUS.STATE.MOVE_TO_BRANCH_POINT,
                "conditions": [self.condition_achieved_and_update_schedules_event]
            },

            {
                "trigger": SIM_BUS.TRIGGER.REQUEST_SCHEDULES,
                "source": SIM_BUS.STATE.MOVE_TO_BRANCH_POINT, "dest": SIM_BUS.STATE.REQUEST_THROUGH_SCHEDULES,
                "conditions": [self.condition_no_need_to_via_and_update_schedue_event]
            },
            {
                "trigger": SIM_BUS.TRIGGER.REQUEST_SCHEDULES,
                "source": SIM_BUS.STATE.MOVE_TO_BRANCH_POINT, "dest": SIM_BUS.STATE.REQUEST_VIA_SCHEDULES,
                "conditions": [self.condition_need_to_via_and_update_schedue_event]
            },

            {
                "trigger": SIM_BUS.TRIGGER.MOVE,
                "source": SIM_BUS.STATE.REQUEST_THROUGH_SCHEDULES, "dest": SIM_BUS.STATE.MOVE_TO_JUNCTION,
                "conditions": [self.condition_schedules_length_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_BUS.TRIGGER.MOVE,
                "source": SIM_BUS.STATE.REQUEST_VIA_SCHEDULES, "dest": SIM_BUS.STATE.MOVE_TO_BUS_STOP,
                "conditions": [self.condition_schedules_length_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_BUS.TRIGGER.STOP,
                "source": SIM_BUS.STATE.MOVE_TO_BUS_STOP, "dest": SIM_BUS.STATE.STOP_FOR_DISCHARGING_AND_TAKING_UP,
                "conditions": [self.condition_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_BUS.TRIGGER.MOVE,
                "source": SIM_BUS.STATE.STOP_FOR_DISCHARGING_AND_TAKING_UP, "dest": SIM_BUS.STATE.MOVE_TO_JUNCTION,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
            {
                "trigger": SIM_BUS.TRIGGER.MOVE,
                "source": SIM_BUS.STATE.MOVE_TO_JUNCTION, "dest": SIM_BUS.STATE.MOVE_TO_SELECT_POINT,
                "conditions": [self.condition_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_BUS.TRIGGER.STAND_BY,
                "source": SIM_BUS.STATE.STOP_FOR_DISCHARGING_AND_TAKING_UP, "dest": SIM_BUS.STATE.STAND_BY,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
        ])
        return machine

    @staticmethod
    def condition_need_to_via(schedules, user_statuses):
        logger.pp(user_statuses)

        target_bus_stops = Target.get_same_group_targets_in_targets("spot", schedules[0].targets)
        for target_bus_stop in target_bus_stops:
            waiting_user_statuses = list(
                filter(lambda x: x.state == SIM_BUS_USER.STATE.WAITING, user_statuses.values()))
            for user_status in waiting_user_statuses:
                target_waiting_bus_stop = Target.get_same_group_targets_in_targets(
                    SIM_BUS_USER.TARGET_GROUP.START_BUS_STOP, user_status.trip_schedules[0].targets)[0]
                if target_bus_stop.id == target_waiting_bus_stop.id:
                    return True
            stop_requested_user_statuses = list(
                filter(lambda x: x.schedule.event == SIM_BUS_USER.TRIGGER.REQUEST_STOP, user_statuses.values()))
            for user_status in stop_requested_user_statuses:
                target_waiting_bus_stop = Target.get_same_group_targets_in_targets(
                    SIM_BUS_USER.TARGET_GROUP.GOAL_BUS_STOP, user_status.trip_schedules[0].targets)[0]
                if target_bus_stop.id == target_waiting_bus_stop.id:
                    return True
        return False

    def condition_no_need_to_via(self, schedules, user_statuses):
        return not self.condition_need_to_via(schedules, user_statuses)

    @staticmethod
    def after_state_change_update_schedule_event(schedules, new_event):
        schedules[0].event = new_event
        return True

    def condition_achieved_and_update_schedules_event(self, current_time, schedules):
        if self.condition_achieved():
            self.after_state_change_update_schedules(current_time, schedules)
            self.after_state_change_update_schedule_event(schedules, SIM_BUS.TRIGGER.REQUEST_SCHEDULES)
            return True
        return False

    def condition_no_need_to_via_and_update_schedue_event(self, schedules, user_statuses):
        if self.condition_no_need_to_via(schedules, user_statuses):
            self.after_state_change_update_schedule_event(schedules, SIM_BUS.TRIGGER.MOVE)
            return True
        return False

    def condition_need_to_via_and_update_schedue_event(self, schedules, user_statuses):
        if self.condition_need_to_via(schedules, user_statuses):
            self.after_state_change_update_schedule_event(schedules, SIM_BUS.TRIGGER.MOVE)
            return True
        return False

    def condition_schedules_length_achieved_and_update_schedules(self, current_time, schedules):
        if 1 < len(schedules):
            if self.condition_achieved():
                self.after_state_change_update_schedules(current_time, schedules)
                return True
        return False

    def get_user_statuses_and_lock(self):
        self.user_statuses_lock.acquire()
        return deepcopy(self.user_statuses)

    def set_user_statuses_and_unlock(self, user_statuses):
        self.user_statuses.clear()
        self.user_statuses.update(user_statuses)
        self.user_statuses_lock.release()

    def update_status_schedule(self):
        if self.status.schedule.event == SIM_BUS.TRIGGER.STAND_BY:
            self.status.schedule.period.end = self.schedules[0].period.end

    def update_status(self):
        schedules = self.get_schedules_and_lock()
        user_statuses = self.get_user_statuses_and_lock()

        # logger.info(len(schedules), self.state_machine.state, self.status.location, self.status.schedule.route)

        if self.state_machine.state in [
            SIM_BUS.STATE.MOVE_TO_CIRCULAR_ROUTE,
            SIM_BUS.STATE.MOVE_TO_SELECT_POINT,
            SIM_BUS.STATE.MOVE_TO_BRANCH_POINT,
            SIM_BUS.STATE.REQUEST_THROUGH_SCHEDULES,
            SIM_BUS.STATE.REQUEST_VIA_SCHEDULES,
            SIM_BUS.STATE.MOVE_TO_BUS_STOP,
            SIM_BUS.STATE.MOVE_TO_JUNCTION,
        ]:
            self.update_pose()
            self.update_route()
            self.update_velocity()

        if 1 < len(schedules):
            current_time = time()
            next_event = schedules[1].event

            # logger.pp((self.status.state, next_event, self.status.location))

            if next_event == SIM_BUS.TRIGGER.MOVE:
                self.state_machine.move(current_time, schedules)
            elif next_event == SIM_BUS.TRIGGER.STOP:
                self.state_machine.stop(current_time, schedules)
            elif next_event == SIM_BUS.TRIGGER.STAND_BY:
                self.state_machine.stand_by(current_time, schedules)
            else:
                pass

        elif 1 == len(schedules):
            current_event = schedules[0].event

            # logger.pp((self.status.state, current_event, self.status.location, user_statuses))

            if current_event == SIM_BUS.TRIGGER.REQUEST_SCHEDULES:
                self.state_machine.request_schedules(schedules, user_statuses)

        self.set_user_statuses_and_unlock(user_statuses)
        self.set_schedules_and_unlock(schedules)
