#!/usr/bin/env python
# coding: utf-8

from ams.structures import Target as Structure
from ams.structures import Targets as Structures


class Target(object):

    @staticmethod
    def new_target(_id, group):
        return Structure.new_data(
            id=_id,
            group=group
        )

    @staticmethod
    def new_node_target(node):
        if isinstance(node, type):
            return Target.new_target(None, node.__name__)
        else:
            return Target.new_target(node.event_loop_id, node.__class__.__name__)

    validate_target = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def new_targets(targets):
        return Structures.new_data(targets)

    @staticmethod
    def is_same_id(target1, target2):
        return None not in [target1, target2] and target1.id == target2.id

    @staticmethod
    def is_same_group(target1, target2):
        return None not in [target1, target2] and target1.group == target2.group

    @staticmethod
    def is_same(target1, target2):
        return Target.is_same_id(target1, target2) and Target.is_same_group(target1, target2)

    @staticmethod
    def get_same_group_targets_in_targets(group, targets):
        return list(filter(lambda x: x is not None and x.group == group, targets))
