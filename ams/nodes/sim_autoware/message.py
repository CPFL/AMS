#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import MessageHeader
from ams.nodes.base import Message as BaseMessage
from ams.nodes.sim_autoware import Structure


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


class Message(BaseMessage):
    Status = StatusMessage
    CurrentPose = Structure.Status.CurrentPose
    ClosestWaypoint = Structure.Status.ClosestWaypoint
    DecisionMakerState = Structure.Status.DecisionMakerState
