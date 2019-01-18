#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures import Pose, Target, MessageHeader


topic = {
    "CATEGORIES": {
        "REQUEST_GET_CONFIG": ["request", "get_config"],
        "RESPONSE_GET_CONFIG": ["response", "get_config"],
        "REQUEST_GET_STATUS": ["request", "get_status"],
        "RESPONSE_GET_STATUS": ["response", "get_status"],
    }
}

const = {
    "NODE_NAME": "device",
    "ROLE_NAME": "device",
    "TOPIC": topic,
    "REQUEST_MESSAGE_TIMEOUT": 2.0,
    "RESPONSE_MESSAGE_TIMEOUT": 2.0,
}

CONST = get_namedtuple_from_dict("CONST", const)


config_template = {
    "target_self": Target.get_template(),
    "pose": Pose.get_template()
}

config_schema = {
    "target_self": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": False,
        "nullable": False,
    }
}


class Config(get_structure_superclass(config_template, config_schema)):
    Pose = Pose


status_template = {
    "state": "default",
    "updated_at": 0.0
}

status_schema = {
    "state": {
        "type": "string",
        "required": False,
        "nullable": True,
    },
    "updated_at": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Status(get_structure_superclass(status_template, status_schema)):
    pass


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
    "body": Config.get_template()
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
        "schema": Config.get_schema(),
        "required": True,
        "nullable": False
    }
}


class ConfigMessage(get_structure_superclass(config_message_template, config_message_schema)):
    pass


status_message_template = {
    "header": MessageHeader.get_template(),
    "body": Status.get_template()
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
        "schema": Status.get_schema(),
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


class EventLoop(object):
    CONST = CONST
    Config = Config
    Status = Status
    Message = Message
