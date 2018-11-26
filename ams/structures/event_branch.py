#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Events


event_branch_template = {
    "key": {
        "type": "common",
        "index": 0
    },
    "common": Events.get_template(),
    "main": Events.get_template(),
    "sub": Events.get_template(),
}

event_branch_schema = {
    "key": {
        "type": "dict",
        "schema": {
            "type": {
                "type": "string",
                "required": True,
                "nullable": False
            },
            "index": {
                "type": "integer",
                "required": True,
                "nullable": False
            }
        }
    },
    "common": Events.get_schema(),
    "main": Events.get_schema(),
    "sub": Events.get_schema(),
}


class EventBranch(get_structure_superclass(event_branch_template, event_branch_schema)):
    pass


event_branches_template = [EventBranch.get_template()]

event_branches_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": EventBranch.get_schema(),
        "required": True,
        "nullable": False,
    },
    "minlength": 1
}


class EventBranches(get_structure_superclass(event_branches_template, event_branches_schema)):
    pass
