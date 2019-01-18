#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import Target, MessageHeader, RoutePoint, EventLoop, Autoware

topic = {
    "CATEGORIES": {},
    "STATE_CMD": "/state_cmd",
    "LANE_ARRAY": "/based/lane_waypoints_raw",
    "STOP_WAYPOINT_INDEX": "/state/stopline_wpidx"
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
    "TOPIC": topic,
    "KEY_PARTS": {
        "LANE_ARRAY_ID_ROUTE_CODE": "lane_array_id_route_code",
        "ROUTE_CODE_LANE_ARRAY_ID": "route_code_lane_array_id"
    }
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


decision_maker_state_message_template = {
    "header": MessageHeader.get_template(),
    "body": Autoware.ROSMessage.DecisionMakerState.get_template()
}


decision_maker_state_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "dict",
        "schema": Autoware.ROSMessage.DecisionMakerState.get_schema(),
        "required": True,
        "nullable": False
    }
}


class DecisionMakerStateMessage(get_structure_superclass(
        decision_maker_state_message_template, decision_maker_state_message_schema)):
    ROSMessage = Autoware.ROSMessage.DecisionMakerState


class Message(EventLoop.Message):
    Config = ConfigMessage
    LaneArray = Autoware.ROSMessage.LaneArray
    StateCMD = Autoware.ROSMessage.StateCMD
    StopWaypointIndex = Autoware.ROSMessage.StopWaypointIndex
    RoutePoint = RoutePointMessage
    DecisionMakerState = DecisionMakerStateMessage


class AutowareInterface(EventLoop):
    CONST = CONST
    Config = Config
    Message = Message
