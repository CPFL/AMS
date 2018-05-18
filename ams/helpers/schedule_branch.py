#!/usr/bin/env python
# coding: utf-8

from ams.structures import ScheduleBranch as Structure


class ScheduleBranch(object):

    @staticmethod
    def new_schedule_branch(common_schedules, main_schedules=None, sub_schedules=None, key=None):
        return Structure.new_data(
            key={"type": "common", "index": 0} if key is None else key,
            common=common_schedules,
            main=main_schedules,
            sub=sub_schedules
        )

    validate_selective_schedules = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def get_current_schedule(schedule_branch):
        return schedule_branch[schedule_branch.key.type][schedule_branch.key.index]
