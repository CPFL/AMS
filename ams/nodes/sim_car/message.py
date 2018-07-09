#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import MessageHeader, Location
from ams.nodes.vehicle import Message as VehicleMessage


location_message_template = MessageHeader.get_template()
location_message_template.update({
    "location": Location.get_template()
})

location_message_schema = MessageHeader.get_schema()
location_message_schema.update({
    "location": {
        "type": "dict",
        "schema": Location.get_schema(),
        "required": True,
        "nullable": True,
    }
})


class LocationMessage(get_structure_superclass(location_message_template, location_message_schema)):
    pass


class Message(VehicleMessage):
    Location = LocationMessage
