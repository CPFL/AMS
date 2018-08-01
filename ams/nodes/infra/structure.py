#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Pose, Target


config_template = {
    "activation": None,
    "target_controller": None,
    "pose": Pose.get_template(),
}

config_schema = {
    "activation": {
        "type": "string",
        "required": True,
        "nullable": True
    },
    "target_controller": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": True
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": True,
        "nullable": True,
    }
}


class Config(get_structure_superclass(config_template, config_schema)):
    Target = Target


status_template = {
    "state": "default",
    "schedule_id": "s0",
    "updated_at": 0.0
}

status_schema = {
    "state": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "schedule_id": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "updated_at": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Status(get_structure_superclass(status_template, status_schema)):
    Pose = Pose


class Structure(object):
    Config = Config
    Status = Status
