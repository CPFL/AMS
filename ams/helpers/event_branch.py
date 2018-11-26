#!/usr/bin/env python
# coding: utf-8

from ams.structures import EventBranch as Structure


class EventBranch(object):

    @staticmethod
    def new_event_branch(common_events, main_events=None, sub_events=None, key=None):
        return Structure.new_data(
            key={"type": "common", "index": 0} if key is None else key,
            common=common_events,
            main=main_events,
            sub=sub_events
        )

    validate_selective_events = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def get_current_event(event_branch):
        return event_branch[event_branch.key.type][event_branch.key.index]
