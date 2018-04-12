#!/usr/bin/env python
# coding: utf-8

from time import time

from ams import StateMachine
from ams.nodes import User
from ams.structures import USER, SIM_TAXI_USER


class SimTaxiUser(User):

    CONST = SIM_TAXI_USER

    def __init__(self, _id, name, dt=1.0):
        super().__init__(_id, name, dt)

        self.state_machine = self.get_state_machine()

    def get_state_machine(self, initial_state=USER.STATE.LOG_IN):
        machine = StateMachine(
            states=list(USER.STATE)+list(SIM_TAXI_USER.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": SIM_TAXI_USER.TRIGGER.REQUEST,
                "source": USER.STATE.LOG_IN, "dest": SIM_TAXI_USER.STATE.CALLING,
                "conditions": [self.after_state_change_update_schedules]
            },
            {
                "trigger": SIM_TAXI_USER.TRIGGER.WAIT,
                "source": SIM_TAXI_USER.STATE.CALLING, "dest": SIM_TAXI_USER.STATE.WAITING,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
            {
                "trigger": SIM_TAXI_USER.TRIGGER.GET_ON,
                "source": SIM_TAXI_USER.STATE.WAITING, "dest": SIM_TAXI_USER.STATE.GETTING_ON,
                "conditions": [self.condition_time_limit_and_update_schedules_and_time_limit]
            },
            {
                "trigger": SIM_TAXI_USER.TRIGGER.GOT_ON,
                "source": SIM_TAXI_USER.STATE.GETTING_ON, "dest": SIM_TAXI_USER.STATE.GOT_ON,
                "conditions": [self.after_state_change_update_schedules]
            },
            {
                "trigger": SIM_TAXI_USER.TRIGGER.MOVE_VEHICLE,
                "source": SIM_TAXI_USER.STATE.GOT_ON, "dest": SIM_TAXI_USER.STATE.MOVING,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
            {
                "trigger": SIM_TAXI_USER.TRIGGER.GET_OUT,
                "source": SIM_TAXI_USER.STATE.MOVING, "dest": SIM_TAXI_USER.STATE.GETTING_OUT,
                "conditions": [self.condition_time_limit_and_update_schedules_and_time_limit]
            },
            {
                "trigger": SIM_TAXI_USER.TRIGGER.GOT_OUT,
                "source": SIM_TAXI_USER.STATE.GETTING_OUT, "dest": SIM_TAXI_USER.STATE.GOT_OUT,
                "conditions": [self.condition_time_limit_and_update_schedules_and_time_limit]
            },
            {
                "trigger": USER.TRIGGER.LOG_OUT,
                "source": SIM_TAXI_USER.STATE.GOT_OUT, "dest": USER.STATE.LOG_OUT,
                "conditions": [self.after_state_change_update_schedules]
            },
        ])
        return machine

    def condition_time_limit_and_update_schedules_and_time_limit(self, current_time, schedules, duration):
        if self.condition_time_limit(current_time, schedules):
            self.after_state_change_update_schedules(current_time, schedules)
            self.after_state_change_update_time_limit(current_time, duration)
            return True
        return False

    def update_status_schedule(self):
        self.status.schedule.period.end = self.schedules[0].period.end
        pass

    def update_status(self):
        schedules = self.get_schedules_and_lock()

        if 1 < len(schedules):
            current_time = time()
            next_event = schedules[1].event

            if next_event == SIM_TAXI_USER.TRIGGER.REQUEST:
                self.state_machine.request(current_time, schedules)
            elif next_event == SIM_TAXI_USER.TRIGGER.WAIT:
                self.state_machine.wait(current_time, schedules)
            elif next_event == SIM_TAXI_USER.TRIGGER.GET_ON:
                self.state_machine.get_on(current_time, schedules, 5)
            elif next_event == SIM_TAXI_USER.TRIGGER.GOT_ON:
                self.state_machine.got_on(current_time, schedules)
            elif next_event == SIM_TAXI_USER.TRIGGER.MOVE_VEHICLE:
                self.state_machine.move_vehicle(current_time, schedules)
            elif next_event == SIM_TAXI_USER.TRIGGER.GET_OUT:
                self.state_machine.get_out(current_time, schedules, 5)
            elif next_event == SIM_TAXI_USER.TRIGGER.GOT_OUT:
                self.state_machine.got_out(current_time, schedules, 1)
            elif next_event == USER.TRIGGER.LOG_OUT:
                self.state_machine.log_out(current_time, schedules)
            else:
                pass

        self.set_schedules_and_unlock(schedules)
