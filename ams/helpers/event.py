#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

from ams.structures import Event as Structure
from ams.structures import Events as Structures
from ams.structures import Period


class Event(object):

    @staticmethod
    def new_event(targets, name, _id=None, start_time=None, end_time=None, route_code=None):
        event = Structure.new_data(
            id=_id if _id is not None else Event.get_id(),
            targets=targets,
            name=name,
            period=Period.new_data(
                start=start_time,
                end=end_time
            ) if None not in [start_time, end_time] else None
        )
        if route_code is not None:
            event.route_code = route_code
        return event

    @staticmethod
    def new_events(events):
        return Structures.new_data(events)

    validate_event = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def get_id():
        return str(uuid())

    @staticmethod
    def get_time():
        return time()

    @staticmethod
    def get_shifted_events(events, dt):
        for event in events:
            event.period.start += dt
            event.period.end += dt
        return events

    @staticmethod
    def get_merged_events(events_1, events_2):
        # todo: check enable
        events_1.extend(events_2)
        return events_1

    @staticmethod
    def get_event_from_cycle(targets, cycle, start_time):
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
        return Event.new_event(targets, state, start_time, end_time)

    @staticmethod
    def get_event_by_id(events, event_id):
        filtered_events = list(filter(lambda x: x.id == event_id, events))
        if len(filtered_events) == 1:
            return filtered_events[0]
        return None

    @staticmethod
    def get_event_index_by_event_id(events, event_id):
        return list(map(lambda x: x.id, events)).index(event_id)

    @staticmethod
    def get_next_event_by_current_event_id(events, current_event_id):
        next_event_index = Event.get_event_index_by_event_id(events, current_event_id) + 1
        if next_event_index < len(events):
            return events[next_event_index]
        return None
