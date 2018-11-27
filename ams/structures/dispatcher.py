#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import Target, Targets, Events, MessageHeader, EventLoop


topic = {
    "CATEGORIES": {}
}
topic["CATEGORIES"].update(event_loop_const["TOPIC"]["CATEGORIES"])
topic["CATEGORIES"].update({
    "CONFIG": ["config"],
    "STATUS": ["status"],
    "EVENTS": ["events"],
    "TRANSPORTATION_CONFIG": ["transportation", "config"],
    "TRANSPORTATION_STATUS": ["transportation", "status"]
})

const = {}
const.update(event_loop_const)
const.update({
    "NODE_NAME": "dispatcher",
    "ROLE_NAME": "dispatcher",
    "TOPIC": topic,
    "TRANSPORTATION": {
        "EVENT": {
            "CHANGE_ROUTE": "change_route",
            "SEND_LANE_ARRAY": "send_lane_array"
        }
    }
})

CONST = get_namedtuple_from_dict("CONST", const)


config_template = EventLoop.Config.get_template()
config_template.update({
    "targets": Targets.get_template()
})

config_schema = EventLoop.Config.get_schema()
config_schema.update({
    "targets": Targets.get_schema()
})


class Config(get_structure_superclass(config_template, config_schema)):
    Targets = Targets


transportation_config_template = {
    "targets": Targets.get_template(),
    "events": [{
        "name": "start_schedule",
        "duration": 1,
        "route_code": "0:0>1:1"
    }]
}

transportation_config_schema = {
    "targets": Targets.get_schema(),
    "events": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": {
                "name": {
                    "type": "string",
                    "required": True,
                    "nullable": False
                },
                "duration": {
                    "type": "number",
                    "required": False,
                    "nullable": False
                },
                "route_code": {
                    "type": "string",
                    "required": False,
                    "nullable": False
                }
            },
            "required": True,
            "nullable": False
        },
        "required": False,
        "nullable": True,
        "minlength": 0
    }
}


class TransportationConfig(get_structure_superclass(transportation_config_template, transportation_config_schema)):
    Targets = Targets


transportation_status_template = {
    "state": "s0",
    "updated_at": 0.0,
    "vehicle_events": Events.get_template()
}

transportation_status_schema = {
    "state": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "updated_at": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "vehicle_events": Events.get_schema()
}


class TransportationStatus(get_structure_superclass(transportation_status_template, transportation_status_schema)):
    Targets = Targets


transportation_statuses_template = [TransportationStatus.get_template()]

transportation_statuses_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": TransportationStatus.get_schema(),
        "required": True,
        "nullable": False,
    },
    "nullable": True,
    "minlength": 0
}


class TransportationStatuses(
        get_structure_superclass(transportation_statuses_template, transportation_statuses_schema)):
    TransportationStatus = TransportationStatus


status_template = EventLoop.Status.get_template()
status_template.update({
    "transportation_statuses": TransportationStatuses.get_template()
})

status_schema = EventLoop.Status.get_schema()
status_schema.update({
    "transportation_statuses": TransportationStatuses.get_schema()
})


class Status(get_structure_superclass(status_template, status_schema)):
    TransportationStatuses = TransportationStatuses


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


transportation_status_message_template = {
    "header": MessageHeader.get_template(),
    "body": TransportationStatus.get_template()
}

transportation_status_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "dict",
        "schema": TransportationStatus.get_schema(),
        "required": True,
        "nullable": False
    }
}


class TransportationStatusMessage(
        get_structure_superclass(transportation_status_message_template, transportation_status_message_schema)):
    pass


events_message_template = {
    "header": MessageHeader.get_template(),
    "body": {
        "target": Target.get_template(),
        "events": Events.get_template()
    }
}

events_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "dict",
        "schema": {
            "target": {
                "type": "dict",
                "schema": Target.get_schema(),
                "required": True,
                "nullable": False
            },
            "events": Events.get_schema()
        },
        "required": True,
        "nullable": False
    }
}


class EventsMessage(
        get_structure_superclass(events_message_template, events_message_schema)):
    pass


class Message(EventLoop.Message):
    Config = ConfigMessage
    Status = StatusMessage
    TransportationStatus = TransportationStatusMessage
    Events = EventsMessage


class Dispatcher(EventLoop):
    CONST = CONST
    Config = Config
    TransportationConfig = TransportationConfig
    Status = Status
    TransportationStatus = TransportationStatus
    TransportationStatuses = TransportationStatuses
    Message = Message
