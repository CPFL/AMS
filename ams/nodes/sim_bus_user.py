#!/usr/bin/env python
# coding: utf-8

from time import time
from copy import deepcopy

from ams import StateMachine
from ams.helpers import Topic, Target
from ams.nodes import User
from ams.messages import VehicleStatus
from ams.structures import SIM_BUS, VEHICLE, SIM_BUS_USER, USER


class SimBusUser(User):

    CONST = SIM_BUS_USER

    def __init__(self, _id, name, dt=1.0):
        super().__init__(_id, name, dt)

        self.state_machine = self.get_state_machine()

        self.target_start_bus_stop = None
        self.target_goal_bus_stop = None
        self.vehicle_id = None

        self.vehicle_statuses = self.manager.dict()
        self.vehicle_statuses_lock = self.manager.Lock()

        self.set_subscriber(
            topic=Topic.get_topic(
                from_target=Target.new_target(SIM_BUS.NODE_NAME, None),
                categories=VEHICLE.TOPIC.CATEGORIES.STATUS,
                use_wild_card=True
            ),
            callback=self.update_vehicle_status,
            structure=VehicleStatus
        )

    def update_vehicle_status(self, _client, _userdata, topic, vehicle_status):
        vehicle_id = Topic.get_from_id(topic)

        self.vehicle_statuses_lock.acquire()
        self.vehicle_statuses[vehicle_id] = vehicle_status
        self.vehicle_statuses_lock.release()

    def get_state_machine(self, initial_state=USER.STATE.LOG_IN):
        machine = StateMachine(
            states=list(USER.STATE)+list(SIM_BUS_USER.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": SIM_BUS_USER.TRIGGER.WAIT,
                "source": USER.STATE.LOG_IN, "dest": SIM_BUS_USER.STATE.WAITING,
                "conditions": [self.after_state_change_update_schedules]
            },
            {
                "trigger": SIM_BUS_USER.TRIGGER.GET_ON,
                "source": SIM_BUS_USER.STATE.WAITING, "dest": SIM_BUS_USER.STATE.GETTING_ON,
                "conditions": [self.condition_bus_arrived_at_start_and_update_schedules]
            },
            {
                "trigger": SIM_BUS_USER.TRIGGER.GOT_ON,
                "source": SIM_BUS_USER.STATE.GETTING_ON, "dest": SIM_BUS_USER.STATE.GOT_ON,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
            {
                "trigger": SIM_BUS_USER.TRIGGER.MOVE_VEHICLE,
                "source": SIM_BUS_USER.STATE.GOT_ON, "dest": SIM_BUS_USER.STATE.MOVING,
                "conditions": [self.condition_bus_moving_and_update_schedules]
            },
            {
                "trigger": SIM_BUS_USER.TRIGGER.REQUEST_STOP,
                "source": SIM_BUS_USER.STATE.MOVING, "dest": SIM_BUS_USER.STATE.READY_TO_GET_OUT,
                "conditions": [self.condition_bus_approached_target_bus_stop_and_update_schedules]
            },
            {
                "trigger": SIM_BUS_USER.TRIGGER.GET_OUT,
                "source": SIM_BUS_USER.STATE.READY_TO_GET_OUT, "dest": SIM_BUS_USER.STATE.GETTING_OUT,
                "conditions": [self.condition_bus_arrived_at_goal_and_update_schedules]
            },
            {
                "trigger": SIM_BUS_USER.TRIGGER.GOT_OUT,
                "source": SIM_BUS_USER.STATE.GETTING_OUT, "dest": SIM_BUS_USER.STATE.GOT_OUT,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
            {
                "trigger": USER.TRIGGER.LOG_OUT,
                "source": SIM_BUS_USER.STATE.GOT_OUT, "dest": USER.STATE.LOG_OUT,
                "conditions": [self.after_state_change_update_schedules]
            },
        ])
        return machine

    @staticmethod
    def condition_bus_arrived_at_target_bus_stop(target_bus_stop, vehicle_status):
        targets_vehicle_statuses = list(filter(
            lambda x: Target.is_same_id(x, target_bus_stop),
            vehicle_status.schedule.targets))
        if 0 < len(targets_vehicle_statuses):
            if vehicle_status.schedule.event == SIM_BUS.TRIGGER.STOP:
                return True
        return False

    def get_vehicle_id_arrived_at_target_bus_stop(self, target_bus_stop, vehicle_statuses):
        for vehicle_id, vehicle_status in vehicle_statuses.items():
            if self.condition_bus_arrived_at_target_bus_stop(target_bus_stop, vehicle_status):
                return vehicle_id
        return None

    @staticmethod
    def condition_bus_moving(vehicle_statuse):
        return vehicle_statuse.state in [
            SIM_BUS.STATE.MOVE_TO_SELECT_POINT,
            SIM_BUS.STATE.REQUEST_THROUGH_SCHEDULES,
            SIM_BUS.STATE.REQUEST_VIA_SCHEDULES,
            SIM_BUS.STATE.MOVE_TO_BRANCH_POINT,
            SIM_BUS.STATE.MOVE_TO_BUS_STOP,
            SIM_BUS.STATE.MOVE_TO_JUNCTION,
            SIM_BUS.STATE.MOVE_TO_PARKING,
        ]

    @staticmethod
    def condition_bus_approached_target_bus_stop(vehicle_status, target_bus_stop):
        return 0 < len(list(filter(
            lambda x: Target.is_same_id(target_bus_stop, x),
            vehicle_status.schedule.targets
        )))

    @staticmethod
    def after_state_change_update_schedule_target(vehicle_id, schedules):
        for i, schedule in enumerate(schedules):
            if schedule.event in [
                SIM_BUS_USER.TRIGGER.GOT_ON,
                SIM_BUS_USER.TRIGGER.MOVE_VEHICLE,
                SIM_BUS_USER.TRIGGER.REQUEST_STOP,
                SIM_BUS_USER.TRIGGER.GET_OUT,
                SIM_BUS_USER.TRIGGER.GOT_OUT,
            ]:
                schedules[i].targets.append(Target.new_target(SIM_BUS.NODE_NAME, vehicle_id))
        return True

    def condition_bus_arrived_at_start_and_update_schedules(
            self, current_time, target_bus_stop, vehicle_statuses, schedules, duration):
        vehicle_id = self.get_vehicle_id_arrived_at_target_bus_stop(target_bus_stop, vehicle_statuses)
        if vehicle_id is not None:
            self.after_state_change_update_schedules(current_time, schedules)
            self.after_state_change_update_time_limit(current_time, duration)
            self.after_state_change_update_schedule_target(vehicle_id, schedules)
            return True
        return False

    def condition_bus_moving_and_update_schedules(self, current_time, schedules, vehicle_status):
        if self.condition_bus_moving(vehicle_status):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        return False

    def condition_bus_approached_target_bus_stop_and_update_schedules(self, current_time, schedules, vehicle_status):
        if self.condition_bus_approached_target_bus_stop(vehicle_status, self.target_goal_bus_stop):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        return False

    def condition_bus_arrived_at_goal_and_update_schedules(
            self, current_time, target_bus_stop, vehicle_status, schedules, duration):
        if self.condition_bus_arrived_at_target_bus_stop(target_bus_stop, vehicle_status):
            self.after_state_change_update_schedules(current_time, schedules)
            self.after_state_change_update_time_limit(current_time, duration)
            return True
        return False

    def get_vehicle_statuses_and_lock(self):
        self.vehicle_statuses_lock.acquire()
        return deepcopy(self.vehicle_statuses)

    def set_vehicle_statuses_and_unlock(self, vehicle_statuses):
        self.vehicle_statuses.clear()
        self.vehicle_statuses.update(vehicle_statuses)
        self.vehicle_statuses_lock.release()

    @staticmethod
    def get_vehicle_id_in_schedule(schedule):
        vehicle_ids = list(map(lambda x: x.id, filter(lambda x: x.group == SIM_BUS.NODE_NAME, schedule.targets)))
        if 1 == len(vehicle_ids):
            return vehicle_ids[0]
        else:
            return None

    def update_status(self):
        if self.target_start_bus_stop is None:
            if self.status.trip_schedules is not None:
                self.target_start_bus_stop = \
                    list(filter(
                        lambda x: x.group == SIM_BUS_USER.TARGET_GROUP.START_BUS_STOP,
                        self.status.trip_schedules[0].targets)
                    )[0]
                self.target_goal_bus_stop = \
                    list(filter(
                        lambda x: x.group == SIM_BUS_USER.TARGET_GROUP.GOAL_BUS_STOP,
                        self.status.trip_schedules[0].targets)
                    )[0]

        schedules = self.get_schedules_and_lock()
        vehicle_statuses = self.get_vehicle_statuses_and_lock()

        current_time = time()
        if 1 < len(schedules):
            next_event = schedules[1].event
            vehicle_id = self.get_vehicle_id_in_schedule(schedules[1])
            vehicle_status = None if vehicle_id is None else vehicle_statuses[vehicle_id]

            if next_event == SIM_BUS_USER.TRIGGER.WAIT:
                self.state_machine.wait(current_time, schedules)
            elif next_event == SIM_BUS_USER.TRIGGER.GET_ON:
                self.state_machine.get_on(current_time, self.target_start_bus_stop, vehicle_statuses, schedules, 3)
            elif next_event == SIM_BUS_USER.TRIGGER.GOT_ON:
                self.state_machine.got_on(current_time, schedules)
            elif next_event == SIM_BUS_USER.TRIGGER.MOVE_VEHICLE:
                self.state_machine.move_vehicle(current_time, schedules, vehicle_status)
            elif next_event == SIM_BUS_USER.TRIGGER.REQUEST_STOP:
                self.state_machine.request_stop(current_time, schedules, vehicle_status)
            elif next_event == SIM_BUS_USER.TRIGGER.GET_OUT:
                self.state_machine.get_out(current_time, self.target_goal_bus_stop, vehicle_status, schedules, 3)
            elif next_event == SIM_BUS_USER.TRIGGER.GOT_OUT:
                self.state_machine.got_out(current_time, schedules)
            elif next_event == USER.TRIGGER.LOG_OUT:
                self.state_machine.log_out(current_time, schedules)
            else:
                pass

        self.set_vehicle_statuses_and_unlock(vehicle_statuses)
        self.set_schedules_and_unlock(schedules)
