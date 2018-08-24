#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.nodes.dispatcher import Structure as DispatcherStructure


config_template = DispatcherStructure.Config.get_template()
config_template.update({
    "stop_waypoint_ids": ["w0"]
})

config_schema = DispatcherStructure.Config.get_schema()
config_schema.update({
    "stop_waypoint_ids": {
        "type": "list",
        "schema": {
            "type": "string",
            "nullable": False
        },
        "required": False,
        "nullable": True,
        "minlength": 0
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    pass


class Structure(DispatcherStructure):
    Config = Config
