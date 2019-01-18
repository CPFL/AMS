#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Converter
from ams.structures import TARGET
from ams.structures import Target as Structure
from ams.structures import Targets as Structures


class Target(object):

    CONST = TARGET
    Structure = Structure

    @staticmethod
    def new_target(group, _id=None):
        return Structure.new_data(
            group=group,
            id=_id,
        )

    @staticmethod
    def new_node_target(node):
        if isinstance(node, type):
            return Target.new_target(Converter.camel_case_to_snake_case(node.__name__), None)
        else:
            return node.target

    validate_target = Structure.validate_data

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

    @staticmethod
    def target_in_targets(target, targets):
        return 0 < len(list(filter(lambda x: Target.is_same(x, target), targets)))

    @staticmethod
    def encode(target):
        return TARGET.DELIMITER.join([
            target.group if target.group is not None else "",
            target.id if target.id is not None else ""
        ])

    @staticmethod
    def decode(target_code):
        return Target.new_target(*target_code.split(TARGET.DELIMITER))
