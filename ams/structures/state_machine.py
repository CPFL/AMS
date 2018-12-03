#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass

template = {
    "initial_state": "s0",
    "variables": {
        "v0": {},
        "v1": 1
    },
    "transitions": [
        {
            "event": "e0",
            "from": "s0",
            "to": "s1",
            "hooks": [
                {
                    "function": "h0",
                    "args": [
                        "v0"
                    ]
                }
            ],
            "conditions": [
                {
                    "function": "c0",
                    "args": [
                        "v1"
                    ],
                    "not": True
                }
            ]
        }
    ]
}

arg_schema = {
    "anyof_type": ["number", "string"],
    "nullable": False
}

hook_schema = {
    "function": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "args": {
        "type": "list",
        "schema": arg_schema,
        "required": True,
        "nullable": False,
        "minlength": 1
    }
}

condition_schema = {
    "function": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "args": {
        "type": "list",
        "schema": arg_schema,
        "required": True,
        "nullable": False,
        "minlength": 1
    },
    "not": {
        "type": "boolean",
        "required": False
    }
}

transition_schema = {
    "event": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "from": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "to": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "hooks": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": hook_schema,
            "nullable": False,
            "minlength": 1
        },
        "required": True,
        "nullable": True
    },
    "conditions": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": condition_schema,
            "nullable": False,
            "minlength": 1
        },
        "required": True,
        "nullable": True
    }
}

schema = {
    "initial_state": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "variables": {
        "type": "dict",
        "required": True,
        "nullable": False
    },
    "transitions": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": transition_schema,
            "nullable": False,
            "minlength": 1
        },
        "required": True,
        "nullable": False
    }
}


class StateMachineResource(get_structure_superclass(template, schema)):
    pass
