#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Schedule

status = {
    "name": "fm0",
    "time": 0.0,
    "state": "default",
    "relations": {"from_id": ["to_id"]}
}

status_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "state": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "relations": {
        "type": "dict",
        "required": True,
        "nullable": True,
        "keyschema": {
            "type": "string"
        },
        "valueschema": {
            "type": "list",
            "valueschema": {
                "schema": {
                    "type": "string",
                    "required": True,
                    "nullable": False
                },
            },
            "minlength": 1
        }
    }
}


class Status(get_base_class(status, status_schema)):
    pass


user_schedules = [Schedule.get_template()]

user_schedules_schema = {
    "type": "list",
    "valueschema": {
        "schema": Schedule.get_schema(),
    }
}


class UserSchedules(get_base_class(user_schedules, user_schedules_schema)):
    pass


vehicle_schedules = [Schedule.get_template()]

vehicle_schedules_schema = {
    "type": "list",
    "valueschema": {
        "schema": Schedule.get_schema(),
    }
}


class VehicleSchedules(get_base_class(vehicle_schedules, vehicle_schedules_schema)):
    pass
