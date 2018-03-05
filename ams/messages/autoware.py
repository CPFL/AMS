#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Pose


current_pose = {
    "time": 0.0,
    "pose": Pose.get_template()
}

current_pose_schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": True,
        "nullable": False
    },
}


class CurrentPose(get_base_class(current_pose, current_pose_schema)):
    pass


closest_waypoint = {
    "time": 0.0,
    "index": 0
}

closest_waypoint_schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "index": {
        "type": "integer",
        "required": True,
        "nullable": False,
    }
}


class ClosestWaypoint(get_base_class(closest_waypoint, closest_waypoint_schema)):
    pass


dicision_maker_states = {
    "time": 0.0,
    "main": "default",
    "accel": "default",
    "steer": "default",
    "behavior": "default"
}

dicision_maker_states_schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "main": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "accel": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "steer": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "behavior": {
        "type": "string",
        "required": True,
        "nullable": False
    }
}


class DecisionMakerStates(get_base_class(dicision_maker_states, dicision_maker_states_schema)):
    pass


lane_array = {
    "time": 0.0,
    "lanes": [{
        "waypoints": [{
            "pose": Pose.get_template(),
            "velocity": 0.0
        }]
    }],
}

lane_array_schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "lanes": {
        "type": "list",
        "valueschema": {
            "type": "dict",
            "schema": {
                "waypoints": {
                    "type": "list",
                    "valueschema": {
                        "type": "dict",
                        "schema": {
                            "pose": {
                                "type": "dict",
                                "schema": Pose.get_schema(),
                                "required": True,
                                "nullable": False
                            },
                            "velocity": {
                                "type": "number",
                                "required": True,
                                "nullable": False
                            }
                        }
                    },
                    "required": True,
                    "nullable": False,
                }
            }
        },
        "required": True,
        "nullable": False,
    }
}


class LaneArray(get_base_class(lane_array, lane_array_schema)):
    pass


state_command = {
    "time": 0.0,
    "state": 13
}

state_command_schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "state": {
        "type": "integer",
        "required": True,
        "nullable": False
    }
}


class StateCommand(get_base_class(state_command, state_command_schema)):
    pass


light_color = {
    "time": 0.0,
    "traffic_light": 0
}

light_color_schema = {
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "traffic_light": {
        "type": "integer",
        "required": True,
        "nullable": False
    }
}


class LightColor(get_base_class(light_color, light_color_schema)):
    pass
