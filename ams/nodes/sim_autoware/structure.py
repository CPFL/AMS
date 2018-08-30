#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import AutowareMessage
from ams.nodes.base.structure import Structure as BaseStructure


config_template = BaseStructure.Config.get_template()
config_template.update({
    "step_size": 1
})

config_schema = BaseStructure.Config.get_schema()
config_schema.update({
    "step_size": {
        "type": "number",
        "required": True,
        "nullable": False
    }
})


class Config(get_structure_superclass(config_template, config_schema)):
    pass


status_template = BaseStructure.Status.get_template()
status_template.update({
    "lane_waypoints_array": AutowareMessage.LaneArray.get_template(),
    "state_cmd": AutowareMessage.StateCMD.get_template(),
    "current_pose": AutowareMessage.CurrentPose.get_template(),
    "closest_waypoint": AutowareMessage.ClosestWaypoint.get_template(),
    "decision_maker_state": AutowareMessage.DecisionMakerState.get_template(),
    "light_color": AutowareMessage.LightColor.get_template()
})

status_schema = BaseStructure.Status.get_schema()
status_schema.update({
    "lane_waypoints_array": {
        "type": "dict",
        "schema": AutowareMessage.LaneArray.get_schema(),
        "required": True,
        "nullable": True,
    },
    "state_cmd": {
        "type": "dict",
        "schema": AutowareMessage.StateCMD.get_schema(),
        "required": True,
        "nullable": True,
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
    },
    "light_color": {
        "type": "dict",
        "schema": AutowareMessage.LightColor.get_schema(),
        "required": True,
        "nullable": True,
    }
})


class Status(get_structure_superclass(status_template, status_schema)):
    LaneArray = AutowareMessage.LaneArray
    StateCMD = AutowareMessage.StateCMD
    CurrentPose = AutowareMessage.CurrentPose
    ClosestWaypoint = AutowareMessage.ClosestWaypoint
    DecisionMakerState = AutowareMessage.DecisionMakerState
    LightColor = AutowareMessage.LightColor


class Structure(BaseStructure):
    Config = Config
    Status = Status
