#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Target, AutowareMessage
from ams.nodes.vehicle import Structure as VehicleStructure

config_template = VehicleStructure.Config.get_template()
config_template.update({
    "target_ros": Target.get_template(),
    "upper_distance_from_stopline": 50.0
})

config_schema = VehicleStructure.Config.get_schema()
config_schema.update({
    "target_ros": {
        "type": "dict",
        "schema": Target.get_schema(),
        "required": True,
        "nullable": False,
    },
    "upper_distance_from_stopline": {
        "type": "number",
        "required": True,
        "nullable": False
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    pass


status_template = VehicleStructure.Status.get_template()
status_template.update({
    "route_code": None,
    "current_pose": AutowareMessage.CurrentPose.get_template(),
    "closest_waypoint": AutowareMessage.ClosestWaypoint.get_template(),
    "decision_maker_state": AutowareMessage.DecisionMakerState.get_template()
})

status_schema = VehicleStructure.Status.get_schema()
status_schema.update({
    "route_code": {
        "type": "string",
        "required": True,
        "nullable": True
    },
    "current_pose": {
        "type": "dict",
        "schema": AutowareMessage.CurrentPose.get_schema(),
        "required": True,
        "nullable": True,
    },
    "closest_waypoint": {
        "type": "dict",
        "schema": AutowareMessage.ClosestWaypoint.get_schema(),
        "required": True,
        "nullable": True,
    },
    "decision_maker_state": {
        "type": "dict",
        "schema": AutowareMessage.DecisionMakerState.get_schema(),
        "required": True,
        "nullable": True
    }
})


class Status(get_structure_superclass(status_template, status_schema)):
    pass


class Structure(VehicleStructure):
    Config = Config
    Status = Status
    ROSMessage = AutowareMessage
