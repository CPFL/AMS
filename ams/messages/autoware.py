#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Pose


current_pose = {
    "name": "a0",
    "time": 0.0,
    "pose": Pose.get_template()
}

current_pose_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
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
    "name": "a0",
    "time": 0.0,
    "index": 0
}

closest_waypoint_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
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


state_overlay_text = {
    "name": "a0",
    "time": 0.0,
    "state": "default"
}

state_overlay_text_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "state": {
        "type": "string",
        "required": True,
        "nullable": False
    }
}


class StateOverlayText(get_base_class(state_overlay_text, state_overlay_text_schema)):
    pass


lane_array = {
    "name": "a0",
    "time": 0.0,
    "lanes": [{
        "waypoints": [{
            "pose": Pose.get_template(),
            "velocity": 0.0
        }]
    }],
}

lane_array_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
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
    "name": "a0",
    "time": 0.0,
    "state": 13
}

state_command_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
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
    "name": "a0",
    "time": 0.0,
    "traffic_light": 0
}

light_color_schema = {
    "name": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
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
