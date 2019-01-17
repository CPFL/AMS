#!/usr/bin/env python
# coding: utf-8

from ams import get_namedtuple_from_dict, get_structure_superclass
from ams.structures.event_loop import const as event_loop_const
from ams.structures import (
    Target, Targets, Schedule, Event, MessageHeader, EventLoop, Vehicle, TrafficSignal, User
)


topic = {
    "CATEGORIES": {}
}
topic["CATEGORIES"].update(event_loop_const["TOPIC"]["CATEGORIES"])
topic["CATEGORIES"].update({
    "CONFIG": ["config"],
    "STATUS": ["status"],
    "SCHEDULE": ["schedule"],
    "EVENT": ["event"],
    "STOP_SIGNAL": ["stop_signal"],
    "VEHICLE_INFO": ["vehicle", "info"]
})

const = {}
const.update(event_loop_const)
const.update({
    "NODE_NAME": "dispatcher",
    "TOPIC": topic,
    "EVENT": {
        "END_NODE": "end_node",
        "CHANGE_SCHEDULE": "change_schedule",
        "RETURN_TO_WAITING_EVENT": "return_to_waiting_event",
    },
})

CONST = get_namedtuple_from_dict("CONST", const)


config_template = EventLoop.Config.get_template()
config_template.update({
    "target_vehicles": Targets.get_template(),
    "target_users": Targets.get_template(),
    "vehicle_schedules": [{
        "id": "s0",
        "events": [
            {
                "id": "e0",
                "name": "send_lane_array",
                "route_code": "0:0>1:1"
            }
        ]
    }]
})

config_schema = EventLoop.Config.get_schema()
config_schema.update({
    "target_vehicles": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": Target.get_schema(),
            "required": True,
            "nullable": False
        },
        "required": True,
        "nullable": False
    },
    "target_users": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": Target.get_schema(),
            "required": True,
            "nullable": False
        },
        "required": False,
        "nullable": True
    },
    "vehicle_schedules": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": {
                "id": {
                    "type": "string",
                    "required": True,
                    "nullable": False
                },
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
                            },
                            "id": {
                                "type": "string",
                                "required": False,
                                "nullable": False
                            }
                        }
                    },
                    "required": True,
                    "nullable": True
                }
            },
            "required": True,
            "nullable": False
        },
        "required": False,
        "nullable": True
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    Targets = Targets


status_template = EventLoop.Status.get_template()
status_template.update({
    "vehicle_status": Vehicle.Status.get_template(),
    "traffic_signal_statuses": [{
        "target": Target.get_template(),
        "status": TrafficSignal.Status.get_template()
    }],
    "user_statuses": [{
        "target": Target.get_template(),
        "status": User.Status.get_template()
    }]
})

status_schema = EventLoop.Status.get_schema()
status_schema.update({
    "vehicle_status": {
        "type": "dict",
        "schema": Vehicle.Status.get_schema(),
        "required": True,
        "nullable": True
    },
    "traffic_signal_statuses": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": {
                "target": {
                    "type": "dict",
                    "schema": Target.get_schema(),
                    "required": True,
                    "nullable": False
                },
                "status": {
                    "type": "dict",
                    "schema": TrafficSignal.Status.get_schema(),
                    "required": True,
                    "nullable": False
                }
            },
            "required": True,
            "nullable": False
        },
        "required": False,
        "nullable": True
    },
    "user_statuses": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": {
                "target": {
                    "type": "dict",
                    "schema": Target.get_schema(),
                    "required": True,
                    "nullable": False
                },
                "status": {
                    "type": "dict",
                    "schema": User.Status.get_schema(),
                    "required": True,
                    "nullable": False
                }
            },
            "required": True,
            "nullable": False
        },
        "required": False,
        "nullable": True
    }
})


class Status(get_structure_superclass(status_template, status_schema)):
    VehicleStatus = Vehicle.Status


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


class ScheduleMessage(
        get_structure_superclass(schedule_message_template, schedule_message_schema)):
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


class EventMessage(
        get_structure_superclass(event_message_template, event_message_schema)):
    pass


signal_message_template = {
    "header": MessageHeader.get_template(),
    "body": {
        "signal": True
    }
}

signal_message_schema = {
    "header": {
        "type": "dict",
        "schema": MessageHeader.get_schema(),
        "required": True,
        "nullable": False
    },
    "body": {
        "type": "boolean",
        "required": True,
        "nullable": False
    }
}


class SignalMessage(
        get_structure_superclass(signal_message_template, signal_message_schema)):
    pass


class Message(EventLoop.Message):
    Config = ConfigMessage
    Status = StatusMessage
    Schedule = ScheduleMessage
    Event = EventMessage
    Signal = SignalMessage
    VehicleInfo = Vehicle.Message.Info


class Dispatcher(EventLoop):
    CONST = CONST
    Config = Config
    Status = Status
    Schedule = Schedule
    Message = Message
