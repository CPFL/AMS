#!/usr/bin/env python
# coding: utf-8

from ams.structures import Target as Structure


class Target(object):

    @staticmethod
    def new_target(instance):
        return Structure.new_data(
            id=instance.event_loop_id,
            node=instance.__class__.__name__
        )

    validate_target = Structure.validatie_data
    get_errors = Structure.get_errors
