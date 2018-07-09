#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Pose, Location, Target


config_template = {
    "activation": None,
    "target_dispatcher": None
}

config_schema = {
    "activation": {
        "type": "string",
        "required": True,
        "nullable": True
    },
    "target_dispatcher": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": True
    }
}


class Config(get_structure_superclass(config_template, config_schema)):
    Target = Target


status_template = {
    "state": "default",
    "schedule_id": "s0",
    "location": Location.get_template(),
    "pose": Pose.get_template(),
    "velocity": 0.0,
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
    "location": {
        "type": "dict",
        "schema": Location.get_schema(),
        "required": True,
        "nullable": True,
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": True,
        "nullable": True,
    },
    "velocity": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "updated_at": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Status(get_structure_superclass(status_template, status_schema)):
    Location = Location
    Pose = Pose


class Structure(object):
    Config = Config
    Status = Status
