#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import MessageHeader, Target, Schedules
from ams.nodes.dispatcher import Structure


config_message_template = MessageHeader.get_template()
config_message_template.update({
    "config": Structure.Config.get_template()
})

config_message_schema = MessageHeader.get_schema()
config_message_schema.update({
    "config": {
        "type": "dict",
        "schema": Structure.Config.get_schema(),
        "required": True,
        "nullable": False
    }
})


class ConfigMessage(get_structure_superclass(config_message_template, config_message_schema)):
    pass


status_message_template = MessageHeader.get_template()
status_message_template.update({
    "status": Structure.Status.get_template()
})

status_message_schema = MessageHeader.get_schema()
status_message_schema.update({
    "status": {
        "type": "dict",
        "schema": Structure.Status.get_schema(),
        "required": True,
        "nullable": False
    }
})


class StatusMessage(get_structure_superclass(status_message_template, status_message_schema)):
    pass


transportation_status_message_template = MessageHeader.get_template()
transportation_status_message_template.update({
    "status": Structure.TransportationStatus.get_template()
})

transportation_status_message_schema = MessageHeader.get_schema()
transportation_status_message_schema.update({
    "status": {
        "type": "dict",
        "schema": Structure.TransportationStatus.get_schema(),
        "required": True,
        "nullable": False
    }
})


class TransportationStatusMessage(
        get_structure_superclass(transportation_status_message_template, transportation_status_message_schema)):
    pass


schedules_message_template = MessageHeader.get_template()
schedules_message_template.update({
    "target": Target.get_template(),
    "schedules": Schedules.get_template()
})

schedules_message_schema = MessageHeader.get_schema()
schedules_message_schema.update({
    "target": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": False,
        "nullable": False
    },
    "schedules": Schedules.get_schema()
})


class SchedulesMessage(
        get_structure_superclass(schedules_message_template, schedules_message_schema)):
    pass


class Message(object):
    Config = ConfigMessage
    Status = StatusMessage
    TransportationStatus = TransportationStatusMessage
    Schedules = SchedulesMessage
