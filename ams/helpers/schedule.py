#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

from ams.structures import Schedule as Structure
from ams.structures import Schedules as Structures
from ams.structures import Period


class Schedule(object):

    @staticmethod
    def new_schedule(targets, event, _id=None, start_time=None, end_time=None, route=None):
        return Structure.new_data(
            id=_id if _id is not None else Schedule.get_id(),
            targets=targets,
            event=event,
            period=Period.new_data(
                start=start_time,
                end=end_time
            ) if None not in [start_time, end_time] else None,
            route=route
        )

    @staticmethod
    def new_schedules(schedules):
        return Structures.new_data(schedules)

    validate_schedule = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def get_id():
        return str(uuid())

    @staticmethod
    def get_time():
        return time()

    @staticmethod
    def get_shifted_schedules(schedules, dt):
        for schedule in schedules:
            schedule.period.start += dt
            schedule.period.end += dt
        return schedules

    @staticmethod
    def get_merged_schedules(schedules_1, schedules_2):
        # todo: check enable
        schedules_1.extend(schedules_2)
        return schedules_1

    @staticmethod
    def get_schedule_from_cycle(targets, cycle, start_time):
        base_time = cycle.time
        period = cycle.period
        phase_time = (start_time - base_time) % period
        state = None
        end_time = start_time
        elapse_time = 0
        for phase in cycle.phases:
            elapse_time += phase.duration
            state = phase.state
            end_time = start_time + (elapse_time - phase_time)
            if phase_time < elapse_time:
                break
        return Schedule.new_schedule(targets, state, start_time, end_time)

    @staticmethod
    def get_schedule_by_id(schedules, schedule_id):
        filtered_schedules = list(filter(lambda x: x.id == schedule_id, schedules))
        if len(filtered_schedules) == 1:
            return filtered_schedules[0]
        return None

    @staticmethod
    def get_schedule_index_by_schedule_id(schedules, schedule_id):
        return list(map(lambda x: x.id, schedules)).index(schedule_id)

    @staticmethod
    def get_next_schedule_by_current_schedule_id(schedules, current_schedule_id):
        next_schedule_index = Schedule.get_schedule_index_by_schedule_id(schedules, current_schedule_id) + 1
        if next_schedule_index < len(schedules):
            return schedules[next_schedule_index]
        return None
