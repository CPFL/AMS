#!/usr/bin/env python
# coding: utf-8

from ams.structures import Target as Structure


class Target(object):

    @staticmethod
    def new_target(node):
        if isinstance(node, type):
            _id = None
            name = node.__name__
        else:
            _id = node.event_loop_id,
            name = node.__class__.__name__
        return Structure.new_data(
            id=_id,
            node=name
        )

    validate_target = Structure.validate_data
    get_errors = Structure.get_errors
