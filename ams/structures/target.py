#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass, get_namedtuple_from_dict


TARGET = get_namedtuple_from_dict("CONST", {
    "DELIMITER": "/"
})

target_template = {
    "id": "uuid",
    "group": "EventLoop"
}

target_schema = {
    "id": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "group": {
        "type": "string",
        "required": True,
        "nullable": False,
    }
}


class Target(get_structure_superclass(target_template, target_schema)):
    pass


targets_template = [Target.get_template()]

targets_schema = {
    "type": "list",
    "valueschema": {
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    },
    "required": False,
    "nullable": True,
    "minlength": 1
}


class Targets(get_structure_superclass(targets_template, targets_schema)):
    pass
