#!/usr/bin/env python
# coding: utf-8

from ams.structures import Target as Structure


class Target(object):

    @staticmethod
    def get_target(instance):
        return Structure.get_data(
            id=instance.event_loop_id,
            node=instance.__class__.__name__
        )

    check_route = Structure.check_data
    get_errors = Structure.get_errors
