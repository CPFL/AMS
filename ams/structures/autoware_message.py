#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass
from ams.structures import Target


timestamp_template = {
    "secs": 0,
    "nsecs": 0
}

timestamp_schema = {
    "secs": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "nsecs": {
        "type": "integer",
        "required": True,
        "nullable": False,
    }
}


class Timestamp(get_structure_superclass(timestamp_template, timestamp_schema)):
    pass


header_template = {
    "seq": 0,
    "stamp": Timestamp.get_template(),
    "frame_id": ""
}

header_schema = {
    "seq": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "stamp": {
        "type": "dict",
        "schema": Timestamp.get_schema(),
        "required": True,
        "nullable": False,
    },
    "frame_id": {
        "type": "string",
        "required": True,
        "nullable": False,
    }
}


class Header(get_structure_superclass(header_template, header_schema)):
    pass


vector3_template = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

vector3_schema = {
    "x": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "y": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "z": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Vector3(get_structure_superclass(vector3_template, vector3_schema)):
    pass


pose_template = {
    "position": Vector3.get_template(),
    "orientation": {
        "w": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    }
}

pose_schema = {
    "position": {
        "type": "dict",
        "schema": Vector3.get_schema(),
        "required": True,
        "nullable": False,
    },
    "orientation": {
        "type": "dict",
        "schema": {
            "w": {
                "type": "number",
                "required": True,
                "nullable": False,
            },
            "x": {
                "type": "number",
                "required": True,
                "nullable": False,
            },
            "y": {
                "type": "number",
                "required": True,
                "nullable": False,
            },
            "z": {
                "type": "number",
                "required": True,
                "nullable": False,
            }
        },
        "required": True,
        "nullable": False,
    }
}


class Pose(get_structure_superclass(pose_template, pose_schema)):
    Vector3 = Vector3


current_pose_template = {
    "header": Header.get_template(),
    "pose": Pose.get_template()
}

current_pose_schema = {
    "header": {
        "type": "dict",
        "schema": Header.get_schema(),
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


class CurrentPose(get_structure_superclass(current_pose_template, current_pose_schema)):
    pass


closest_waypoint_template = {
    "data": 0
}

closest_waypoint_schema = {
    "data": {
        "type": "integer",
        "required": True,
        "nullable": False,
    }
}


class ClosestWaypoint(get_structure_superclass(closest_waypoint_template, closest_waypoint_schema)):
    pass


dicision_maker_states_template = {
    "data": "state"
}

dicision_maker_states_schema = {
    "data": {
        "type": "string",
        "required": True,
        "nullable": False
    }
}


class DecisionMakerState(get_structure_superclass(dicision_maker_states_template, dicision_maker_states_schema)):
    pass


pose_stamped_template = {
    "header": Header.get_template(),
    "pose": Pose.get_template()
}

pose_stamped_schema = {
    "header": {
        "type": "dict",
        "schema": Header.get_schema(),
        "required": True,
        "nullable": False,
    },
    "pose": {
        "type": "dict",
        "schema": Pose.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class PoseStamped(get_structure_superclass(pose_stamped_template, pose_stamped_schema)):
    Header = Header
    Pose = Pose


twist_template = {
    "linear": Vector3.get_template(),
    "angular": Vector3.get_template()
}

twist_schema = {
    "linear": {
        "type": "dict",
        "schema": Vector3.get_schema(),
        "required": True,
        "nullable": False,
    },
    "angular": {
        "type": "dict",
        "schema": Vector3.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class Twist(get_structure_superclass(twist_template, twist_schema)):
    pass


twist_stamped_template = {
    "header": Header.get_template(),
    "twist": Twist.get_template()
}

twist_stamped_schema = {
    "header": {
        "type": "dict",
        "schema": Header.get_schema(),
        "required": True,
        "nullable": False,
    },
    "twist": {
        "type": "dict",
        "schema": Twist.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class TwistStamped(get_structure_superclass(twist_stamped_template, twist_stamped_schema)):
    pass


dtlane_template = {
    "dist": 0.0,
    "dir": 0.0,
    "apara": 0.0,
    "r": 0.0,
    "slope": 0.0,
    "cant": 0.0,
    "lw": 0.0,
    "rw": 0.0
}

dtlane_schema = {
    "dist": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "dir": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "apara": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "r": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "slope": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "cant": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "lw": {
        "type": "number",
        "required": True,
        "nullable": False,
    },
    "rw": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Dtlane(get_structure_superclass(dtlane_template, dtlane_schema)):
    pass


waypoint_state_template = {
    "aid": 0,
    "lanechange_state": 0,
    "steering_state": 0,
    "accel_state": 0,
    "stop_state": 0,
    "event_state": 0
}

waypoint_state_schema = {
    "aid": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "lanechange_state": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "steering_state": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "accel_state": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "stop_state": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "event_state": {
        "type": "integer",
        "required": True,
        "nullable": False,
    }

}


class WaypointState(get_structure_superclass(waypoint_state_template, waypoint_state_schema)):
    pass


waypoint_template = {
    "gid": 0,
    "lid": 0,
    "pose": PoseStamped.get_template(),
    "twist": TwistStamped.get_template(),
    "dtlane": Dtlane.get_template(),
    "change_flag": 0,
    "wpstate": WaypointState.get_template()

}

waypoint_schema = {
    "gid": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "lid": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "pose": {
        "type": "dict",
        "schema": PoseStamped.get_schema(),
        "required": True,
        "nullable": False,
    },
    "twist": {
        "type": "dict",
        "schema": TwistStamped.get_schema(),
        "required": True,
        "nullable": False,
    },
    "dtlane": {
        "type": "dict",
        "schema": Dtlane.get_schema(),
        "required": True,
        "nullable": False,
    },
    "change_flag": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "wpstate": {
        "type": "dict",
        "schema": WaypointState.get_schema(),
        "required": True,
        "nullable": False,
    },
}


class Waypoint(get_structure_superclass(waypoint_template, waypoint_schema)):
    PoseStamped = PoseStamped
    TwistStamped = TwistStamped
    Dtlane = Dtlane
    WaypointState = WaypointState


lane_template = {
    "header": Header.get_template(),
    "increment": 0,
    "lane_id": 0,
    "waypoints": [Waypoint.get_template()]
}

lane_schema = {
    "header": {
        "type": "dict",
        "schema": Header.get_schema(),
        "required": True,
        "nullable": False,
    },
    "increment": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "lane_id": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "waypoints": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": Waypoint.get_schema(),
            "required": True,
            "nullable": False
        },
        "required": True,
        "nullable": False
    }
}


class Lane(get_structure_superclass(lane_template, lane_schema)):
    Header = Header
    Waypoint = Waypoint


lane_array_template = {
    "lanes": [Lane.get_template()]
}

lane_array_schema = {
    "lanes": {
        "type": "list",
        "schema": {
            "type": "dict",
            "schema": Lane.get_schema(),
            "required": True,
            "nullable": False
        },
        "required": True,
        "nullable": False
    }
}


class LaneArray(get_structure_superclass(lane_array_template, lane_array_schema)):
    Lane = Lane


state_cmd_template = {
    "data": "command"
}

state_cmd_schema = {
    "data": {
        "type": "string",
        "required": True,
        "nullable": False
    }
}


class StateCMD(get_structure_superclass(state_cmd_template, state_cmd_schema)):
    pass


light_color_template = {
    "header": Header.get_template(),
    "traffic_light": 0
}

light_color_schema = {
    "header": {
        "type": "dict",
        "schema": Header.get_schema(),
        "required": True,
        "nullable": False
    },
    "traffic_light": {
        "type": "integer",
        "required": True,
        "nullable": False
    }
}


class LightColor(get_structure_superclass(light_color_template, light_color_schema)):
    pass


class AutowareMessage(object):
    Header = Header
    CurrentPose = CurrentPose
    ClosestWaypoint = ClosestWaypoint
    DecisionMakerState = DecisionMakerState
    LaneArray = LaneArray
    StateCMD = StateCMD
    LightColor = LightColor
