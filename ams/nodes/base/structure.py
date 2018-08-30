#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Pose, Target


config_template = {
    "pose": Pose.get_template(),
    "target_roles": {
        "self": Target.get_template()
    }
}

config_schema = {
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": False,
        "nullable": False,
    },
    "target_roles": {
        "type": "dict",
        "keyschema": {
            "type": "string",
            "regex": "[a-z]+",
            "nullable": False
        },
        "valueschema": {
            "type": "dict",
            "schema": Target.get_schema(),
            "nullable": False
        },
        "required": False,
        "nullable": False,
    }
}


class Config(get_structure_superclass(config_template, config_schema)):
    Pose = Pose


status_template = {
    "state": "default",
    "updated_at": 0.0
}

status_schema = {
    "state": {
        "type": "string",
        "required": False,
        "nullable": False,
    },
    "updated_at": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Status(get_structure_superclass(status_template, status_schema)):
    pass


class Structure(object):
    Config = Config
    Status = Status
