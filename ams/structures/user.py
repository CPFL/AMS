#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import Target, Location, MessageHeader, EventLoop, Vehicle


topic = {
    "CATEGORIES": {}
}
topic["CATEGORIES"].update(event_loop_const["TOPIC"]["CATEGORIES"])
topic["CATEGORIES"].update({
    "CONFIG": ["config"],
    "STATUS": ["status"]
})

const = {}
const.update(event_loop_const)
const.update({
    "NODE_NAME": "user",
    "TOPIC": topic,
    "STATE": {
        "CALLING": "Calling",
        "WAITING_VEHICLE": "WaitingVehicle",
        "GOT_ON": "GotOn",
        "GOT_OFF": "GotOff",
        "END": "End"
    }
})

CONST = get_namedtuple_from_dict("CONST", const)


config_template = EventLoop.Config.get_template()
config_template.update({
    "target_dispatcher": Target.get_template()
})

config_schema = EventLoop.Config.get_schema()
config_schema.update({
    "target_dispatcher": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    Target = Target


status_template = EventLoop.Status.get_template()
status_template.update({
    "vehicle_info": Vehicle.Info.get_template(),
    "start_location": Location.get_template(),
    "goal_location": Location.get_template()
})

status_schema = EventLoop.Status.get_schema()
status_schema.update({
    "vehicle_info": {
        "type": "dict",
        "schema": Vehicle.Info.get_schema(),
        "required": True,
        "nullable": True
    },
    "start_location": {
        "type": "dict",
        "schema": Location.get_schema(),
        "required": True,
        "nullable": False
    },
    "goal_location": {
        "type": "dict",
        "schema": Location.get_schema(),
        "required": True,
        "nullable": True
    }
})


class Status(get_structure_superclass(status_template, status_schema)):
    VehicleInfo = Vehicle.Info
    Location = Location


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
    "body": {
        "target": Target.get_template(),
        "status": Status.get_template()
    }
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


class Message(EventLoop.Message):
    Config = ConfigMessage
    Status = StatusMessage


class User(EventLoop):
    CONST = CONST
    Config = Config
    Status = Status
    Message = Message
