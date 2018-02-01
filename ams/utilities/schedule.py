#!/usr/bin/env python
# coding: utf-8

from ams.structures import Schedule as Structure
from ams.structures import Period


class Schedule(object):

    def __init__(self):
        self.__waypoint = None
        self.__route = None

    def set_waypoint(self, waypoint):
        self.__waypoint = waypoint

    def set_route(self, route):
        self.__route = route

    @staticmethod
    def get_schedule(event, start_time, end_time, route=None):
        return Structure.get_data(
            event=event,
            period=Period.get_data(
                start=start_time,
                end=end_time
            ),
            route=route
        )

    check_schedule = Structure.check_data
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
    def get_schedule_from_cycle(cycle, start_time):
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
        return Schedule.get_schedule(state, start_time, end_time)
