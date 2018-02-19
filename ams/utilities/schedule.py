#!/usr/bin/env python
# coding: utf-8

from ams.structures import Schedule as Structure
from ams.structures import Period, Targets


class Schedule(object):

    def __init__(self):
        self.__waypoint = None
        self.__route = None

    def set_waypoint(self, waypoint):
        self.__waypoint = waypoint

    def set_route(self, route):
        self.__route = route

    @staticmethod
    def new_schedule(targets, event, start_time=None, end_time=None, route=None):
        return Structure.new_data(
            targets=targets,
            event=event,
            period=Period.new_data(
                start=start_time,
                end=end_time
            ) if None not in [start_time, end_time] else None,
            route=route
        )

    validate_schedule = Structure.validate_data
    get_errors = Structure.get_errors

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
