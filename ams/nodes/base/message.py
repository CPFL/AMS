#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import MessageHeader
from ams.nodes.base import Structure

request_config_message_template = {
    "header": MessageHeader.get_template()
}

request_config_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    }
}


class RequestConfigMessage(get_structure_superclass(request_config_message_template, request_config_message_schema)):
    pass


config_message_template = {
    "header": MessageHeader.get_template(),
    "body": Structure.Config.get_template()
}

config_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "dict",
        "schema": Structure.Config.get_schema(),
        "required": True,
        "nullable": False
    }
}


class ConfigMessage(get_structure_superclass(config_message_template, config_message_schema)):
    pass


status_message_template = {
    "header": MessageHeader.get_template(),
    "body": Structure.Status.get_template()
}

status_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "dict",
        "schema": Structure.Status.get_schema(),
        "required": True,
        "nullable": False
    }
}


class StatusMessage(get_structure_superclass(status_message_template, status_message_schema)):
    pass


class Message(object):
    Header = MessageHeader
    RequestConfig = RequestConfigMessage
    RequestStatus = RequestConfigMessage
    Config = ConfigMessage
    Status = StatusMessage
