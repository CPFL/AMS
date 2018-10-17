#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import Target, MessageHeader, RoutePoint, EventLoop, Autoware

topic = {
    "CATEGORIES": {},
    "STATE_CMD": "/state_cmd",
    "LANE_ARRAY": "/based/lane_waypoints_raw",
    "LIGHT_COLOR": "/light_color"
}
topic["CATEGORIES"].update(event_loop_const["TOPIC"]["CATEGORIES"])
topic["CATEGORIES"].update({
    "CURRENT_POSE": ["current_pose"],
    "ROUTE_POINT": ["route_point"],
    "DECISION_MAKER_STATE": ["decision_maker", "state"]
})

const = {}
const.update(event_loop_const)
const.update({
    "NODE_NAME": "autoware_interface",
    "ROLE_NAME": "autoware_interface",
    "TOPIC": topic
})

CONST = get_namedtuple_from_dict("CONST", const)


config_template = EventLoop.Config.get_template()
config_template.update({
    "target_autoware": Target.get_template(),
    "target_vehicle": Target.get_template(),
})

config_schema = EventLoop.Config.get_schema()
config_schema.update({
    "target_autoware": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    },
    "target_vehicle": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    Target = Target


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


route_point_message_template = {
    "header": MessageHeader.get_template(),
    "body": RoutePoint.get_template()
}


route_point_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "dict",
        "schema": RoutePoint.get_schema(),
        "required": True,
        "nullable": False
    }
}


class RoutePointMessage(get_structure_superclass(route_point_message_template, route_point_message_schema)):
    RoutePoint = RoutePoint


class Message(EventLoop.Message):
    Config = ConfigMessage
    LaneArray = Autoware.ROSMessage.LaneArray
    StateCMD = Autoware.ROSMessage.StateCMD
    LightColor = Autoware.ROSMessage.LightColor
    RoutePoint = RoutePointMessage


class AutowareInterface(EventLoop):
    CONST = CONST
    Config = Config
    Message = Message
