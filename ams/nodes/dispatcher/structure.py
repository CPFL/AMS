#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Targets, Schedules


config_template = {
    "targets": Targets.get_template(),
    "inactive_api_keys": ["test_api_key"],
    "active_api_keys": []
}

config_schema = {
    "targets": Targets.get_schema(),
    "inactive_api_keys": {
        "type": "list",
        "schema": {
            "type": "string",
            "nullable": False
        },
        "required": True,
        "nullable": False,
        "minlength": 0
    },
    "active_api_keys": {
        "type": "list",
        "schema": {
            "type": "string",
            "nullable": False
        },
        "required": True,
        "nullable": False,
        "minlength": 0
    }
}


class Config(get_structure_superclass(config_template, config_schema)):
    Targets = Targets


transportation_status_template = {
    "targets": Targets.get_template(),
    "state": "s0",
    "updated_at": 0.0,
    "schedules": Schedules.get_template()
}

transportation_status_schema = {
    "targets": Targets.get_schema(),
    "state": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "updated_at": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "schedules": Schedules.get_schema()
}


class TransportationStatus(get_structure_superclass(transportation_status_template, transportation_status_schema)):
    Targets = Targets


transportation_statuses_template = [TransportationStatus.get_template()]

transportation_statuses_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": TransportationStatus.get_schema(),
        "required": True,
        "nullable": False,
    },
    "nullable": True,
    "minlength": 0

}


class TransportationStatuses(
        get_structure_superclass(transportation_statuses_template, transportation_statuses_schema)):
    TransportationStatus = TransportationStatus


status_template = {
    "state": "s0",
    "transportation_statuses": TransportationStatuses.get_template()
}

status_schema = {
    "state": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "transportation_statuses": TransportationStatuses.get_schema()
}


class Status(get_structure_superclass(status_template, status_schema)):
    TransportationStatuses = TransportationStatuses


class Structure(object):
    Config = Config
    TransportationStatus = TransportationStatus
    TransportationStatuses = TransportationStatuses
    Status = Status
