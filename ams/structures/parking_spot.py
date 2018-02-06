#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Polygon, Schedule

template = {
    "polygon": Polygon.get_template(),
    "schedules": [Schedule.get_template()],
}

schema = {
    "polygon": {
        "schema": Polygon.get_schema(),
        "required": True,
        "nullable": False,
    },
    "schedules": {
        "type": "list",
        "valueschema": {
            "schema": Schedule.get_schema(),
            "required": True,
            "nullable": True,
        }
    }
}


class ParkingSpot(get_base_class(template, schema)):
    pass
