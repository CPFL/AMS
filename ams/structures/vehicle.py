#!/usr/bin/env python
# coding: utf-8

from sys import float_info

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import Pose, Location, Target, RoutePoint, MessageHeader, EventLoop, Autoware


topic = {
    "CATEGORIES": {}
}
topic["CATEGORIES"].update(event_loop_const["TOPIC"]["CATEGORIES"])
topic["CATEGORIES"].update({
    "CONFIG": ["config"],
    "STATUS": ["status"],
    "GEOTOPIC": ["geotopic"],
    "STATE_CMD": ["state_cmd"],
    "ROUTE_CODE": ["route_code"],
    "LIGHT_COLOR": ["light_color"]
})

const = {}
const.update(event_loop_const)
const.update({
    "NODE_NAME": "vehicle",
    "ROLE_NAME": "vehicle",
    "TOPIC": topic,
    "STATE": {
        "WAIT_SCHEDULE": "WaitSchedule",
        "WAITING_EVENT": "WaitingEvent",
        "WAITING_SCHEDULE": "WaitingSchedule",
        "REPLACING_SCHEDULE": "ReplacingSchedule",
        "REPLACING_SCHEDULE_SUCCEEDED": "ReplacingScheduleSucceeded",
        "REPLACING_SCHEDULE_FAILED": "ReplacingScheduleFailed",
        "END": "End"
    },
    "ACTIVATION_REQUEST_TIMEOUT": 10.0,
    "FLOAT_MAX": float_info.max,
    "DEFAULT_UPPER_DISTANCE_FROM_STOPLINE": 50.0
})

CONST = get_namedtuple_from_dict("CONST", const)


config_template = EventLoop.Config.get_template()
config_template.update({
    "target_autoware": Target.get_template(),
    "target_dispatcher": Target.get_template(),
    "upper_distance_from_stopline": 50.0
})

config_schema = EventLoop.Config.get_schema()
config_schema.update({
    "target_autoware": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    },
    "target_dispatcher": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": True
    },
    "upper_distance_from_stopline": {
        "type": "number",
        "required": True,
        "nullable": False
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    Target = Target


status_template = EventLoop.Status.get_template()
status_template.update({
    "schedule_id": "s0",
    "event_id": "e0",
    "on_event": False,
    "location": Location.get_template(),
    "pose": Pose.get_template(),
    "current_pose": Autoware.Status.CurrentPose.get_template(),
    "route_point": RoutePoint.get_template(),
    "decision_maker_state": Autoware.Status.DecisionMakerState.get_template()
})

status_schema = EventLoop.Status.get_schema()
status_schema.update({
    "schedule_id": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "event_id": {
        "type": "string",
        "required": True,
        "nullable": True,
    },
    "on_event": {
        "type": "boolean",
        "required": True,
        "nullable": False
    },
    "location": {
        "type": "dict",
        "schema": Location.get_schema(),
        "required": True,
        "nullable": True,
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": True,
        "nullable": True,
    },
    "current_pose": {
        "type": "dict",
        "schema": Autoware.Status.CurrentPose.get_schema(),
        "required": True,
        "nullable": True,
    },
    "route_point": {
        "type": "dict",
        "schema": RoutePoint.get_schema(),
        "required": True,
        "nullable": True,
    },
    "decision_maker_state": {
        "type": "dict",
        "schema": Autoware.Status.DecisionMakerState.get_schema(),
        "required": True,
        "nullable": True
    }
})


class Status(get_structure_superclass(status_template, status_schema)):
    Location = Location
    Pose = Pose


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


route_code_message_template = {
    "header": MessageHeader.get_template(),
    "body":  "0:0>1:1"
}

route_code_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "string",
        "required": True,
        "nullable": False
    }
}


class RouteCodeMessage(get_structure_superclass(route_code_message_template, route_code_message_schema)):
    Header = MessageHeader


class Message(EventLoop.Message):
    Config = ConfigMessage
    Status = StatusMessage
    RouteCode = RouteCodeMessage


class Vehicle(EventLoop):
    CONST = CONST
    Config = Config
    Status = Status
    Message = Message
