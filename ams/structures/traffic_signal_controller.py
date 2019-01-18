#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import Target, Targets, Event, Cycle, Schedule, MessageHeader, EventLoop


topic = {
    "CATEGORIES": {}
}
topic["CATEGORIES"].update(event_loop_const["TOPIC"]["CATEGORIES"])
topic["CATEGORIES"].update({
    "CONFIG": ["config"],
    "STATUS": ["status"],
    "EVENT": ["event"],
    "CYCLE": ["cycle"],
    "SCHEDULE": ["schedule"]
})

const = {}
const.update(event_loop_const)
const.update({
    "NODE_NAME": "traffic_signal_controller",
    "EVENT": {
        "END_NODE": "end_node"
    },
    "TOPIC": topic,
    "EVENT_MESSAGE_TIMEOUT": 5.0
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


status_template = EventLoop.Status.get_template()

status_schema = EventLoop.Status.get_schema()


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


event_message_template = {
    "header": MessageHeader.get_template(),
    "body": {
        "target": Target.get_template(),
        "event": Event.get_template()
    }
}

event_message_schema = {
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
            "event": {
                "type": "dict",
                "schema": Event.get_schema(),
                "required": True,
                "nullable": False
            }
        },
        "required": True,
        "nullable": False
    }
}


class EventMessage(get_structure_superclass(event_message_template, event_message_schema)):
    pass


cycle_message_template = {
    "header": MessageHeader.get_template(),
    "body": {
        "target": Target.get_template(),
        "cycle": Cycle.get_template()
    }
}

cycle_message_schema = {
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
            "cycle": {
                "type": "dict",
                "schema": Cycle.get_schema(),
                "required": True,
                "nullable": False
            }
        },
        "required": True,
        "nullable": False
    }
}


class CycleMessage(get_structure_superclass(cycle_message_template, cycle_message_schema)):
    pass


schedule_message_template = {
    "header": MessageHeader.get_template(),
    "body": {
        "target": Target.get_template(),
        "schedule": Schedule.get_template()
    }
}

schedule_message_schema = {
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
            "schedule": {
                "type": "dict",
                "schema": Schedule.get_schema(),
                "required": True,
                "nullable": False
            }
        },
        "required": True,
        "nullable": False
    }
}


class ScheduleMessage(get_structure_superclass(schedule_message_template, schedule_message_schema)):
    pass


class Message(EventLoop.Message):
    Config = ConfigMessage
    Status = StatusMessage
    Event = EventMessage
    Cycle = CycleMessage
    Schedule = ScheduleMessage


class TrafficSignalController(EventLoop):
    CONST = CONST
    Config = Config
    Status = Status
    Message = Message
