#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import Pose, Cycle, Target, MessageHeader, EventLoop


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
    "NODE_NAME": "traffic_signal",
    "ROLE_NAME": "infra",
    "TOPIC": topic,
    "STATE": {
        "WAITING_EVENT": "WaitingEvent",
        "END": "End"
    },
    "LIGHT_COLOR": {
        "RED": "red",
        "YELLOW": "yellow",
        "GREEN": "green",
        "UNKNOWN": "unknown"
    }

})

CONST = get_namedtuple_from_dict("CONST", const)


config_template = EventLoop.Config.get_template()
config_template.update({
    "target_traffic_signal_controller": Target.get_template(),
    "cycle": Cycle.get_template(),
    "route_code": "0:0>1:1",
    "pose": Pose.get_template()
})

config_schema = EventLoop.Config.get_schema()
config_schema.update({
    "target_traffic_signal_controller": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    },
    "cycle": {
        "type": "dict",
        "schema": Cycle.get_schema(),
        "required": True,
        "nullable": True
    },
    "route_code": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": True,
        "nullable": True
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    Target = Target
    Cycle = Cycle
    Pose = Pose


status_template = EventLoop.Status.get_template()
status_template.update({
    "event": "e0",
    "light_color": "red",
    "light_color_schedule_id": "s0",
    "next_light_color": "green",
    "next_update_time": 0.0
})

status_schema = EventLoop.Status.get_schema()
status_schema.update({
    "event": {
        "type": "string",
        "required": True,
        "nullable": True
    },
    "light_color": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "light_color_schedule_id": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "next_light_color": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "next_update_time": {
        "type": "number",
        "required": True,
        "nullable": True,
    }
})


class Status(get_structure_superclass(status_template, status_schema)):
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


class TrafficSignal(EventLoop):
    CONST = CONST
    Config = Config
    Status = Status
    Message = Message
