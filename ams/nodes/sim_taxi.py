#!/usr/bin/env python
# coding: utf-8

from time import time

from transitions import Machine

from ams import logger
from ams.nodes import SimCar
from ams.structures import SIM_TAXI


class SimTaxi(SimCar):

    CONST = SIM_TAXI

    def __init__(
            self, _id, name, waypoint, arrow, route, intersection, dt=1.0):
        super().__init__(_id, name, waypoint, arrow, route, intersection, dt)

        self.state_machine = self.get_state_machine(SIM_TAXI.STATE.STAND_BY)

    def get_state_machine(self, initial_state):
        machine = Machine(
            states=list(SIM_TAXI.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": SIM_TAXI.TRIGGER.MOVE,
                "source": SIM_TAXI.STATE.STAND_BY, "dest": SIM_TAXI.STATE.MOVE_FOR_PICKING_UP,
                "conditions": [self.after_state_change_update_schedules]
            },
            {
                "trigger": SIM_TAXI.TRIGGER.STOP,
                "source": SIM_TAXI.STATE.MOVE_FOR_PICKING_UP, "dest": SIM_TAXI.STATE.STOP_FOR_PICKING_UP,
                "conditions": [self.condition_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_TAXI.TRIGGER.MOVE,
                "source": SIM_TAXI.STATE.STOP_FOR_PICKING_UP, "dest": SIM_TAXI.STATE.MOVE_FOR_DISCHARGING,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
            {
                "trigger": SIM_TAXI.TRIGGER.STOP,
                "source": SIM_TAXI.STATE.MOVE_FOR_DISCHARGING, "dest": SIM_TAXI.STATE.STOP_FOR_DISCHARGING,
                "conditions": [self.condition_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_TAXI.TRIGGER.STAND_BY,
                "source": SIM_TAXI.STATE.STOP_FOR_DISCHARGING, "dest": SIM_TAXI.STATE.STAND_BY,
                "conditions": [self.condition_time_limit_and_update_schedules]
            },
        ])
        return machine

    def update_status_schedule(self):
        if self.status.schedule.event == SIM_TAXI.TRIGGER.STOP:
            self.status.schedule.period.end = self.schedules[0].period.end
        logger.pp((self.status.schedule, list(self.schedules)))

    def update_status(self):
        schedules = self.get_schedules_and_lock()

        if self.state_machine.state in [SIM_TAXI.STATE.MOVE_FOR_PICKING_UP, SIM_TAXI.STATE.MOVE_FOR_DISCHARGING]:
            self.update_pose()
            self.update_route()
            self.update_velocity()

        if 1 < len(schedules):
            current_time = time()
            next_event = schedules[1].event

            if next_event == SIM_TAXI.TRIGGER.MOVE:
                self.state_machine.move(current_time, schedules)
            elif next_event == SIM_TAXI.TRIGGER.STOP:
                self.state_machine.stop(current_time, schedules)
            elif next_event == SIM_TAXI.TRIGGER.STAND_BY:
                self.state_machine.stand_by(current_time, schedules)
            else:
                pass

        # logger.pp({self.target.id: self.state_machine.state, "schedule_len": len(schedules)})

        self.set_schedules_and_unlock(schedules)
