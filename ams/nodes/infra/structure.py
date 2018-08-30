#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.nodes.base.structure import Structure as BaseStructure


config_template = BaseStructure.Config.get_template()
config_template.update({
    "activation": None,
})

config_schema = BaseStructure.Config.get_schema()
config_schema.update({
    "activation": {
        "type": "string",
        "required": True,
        "nullable": True
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    pass


status_template = BaseStructure.Status.get_template()
status_template.update({
    "schedule_id": "s0",
})

status_schema = BaseStructure.Status.get_schema()
status_schema.update({
    "schedule_id": {
        "type": "string",
        "required": True,
        "nullable": True,
    }
})


class Status(get_structure_superclass(status_template, status_schema)):
    pass


class Structure(BaseStructure):
    Config = Config
    Status = Status
