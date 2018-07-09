#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import MessageHeader
from ams.nodes.dispatcher import Message as DispatcherMessage
from ams.nodes.autoware_dispatcher import Structure


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


class Message(DispatcherMessage):
    Config = ConfigMessage
