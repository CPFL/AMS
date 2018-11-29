#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Events


schedule_template = {
    "id": "0",
    "events": Events.get_template()
}

schedule_schema = {
    "id": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "events": Events.get_schema()
}


class Schedule(get_structure_superclass(schedule_template, schedule_schema)):
    pass
