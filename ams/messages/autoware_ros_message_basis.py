#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Position, Quaternion


template = {
    "secs": 0,
    "nsecs": 0
}

schema = {
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


class TimestampRosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "seq": 0,
    "stamp": TimestampRosMessageBasis.get_template(),
    "frame_id": ""
}

schema = {
    "seq": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "stamp": {
        "type": "dict",
        "schema": TimestampRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
    "frame_id": {
        "type": "string",
        "required": True,
        "nullable": False,
    }
}


class HeaderRosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "position": Position.get_template(),
    "orientation": Quaternion.get_template()
}

schema = {
    "position": {
        "type": "dict",
        "schema": Position.get_schema(),
        "required": True,
        "nullable": False,
    },
    "orientation": {
        "type": "dict",
        "schema": Quaternion.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class PoseRosMessageBasis(get_base_class(template, schema)):
    pass


current_pose = {
    "header": HeaderRosMessageBasis.get_template(),
    "pose": PoseRosMessageBasis.get_template()
}

current_pose_schema = {
    "header": {
        "type": "dict",
        "schema": HeaderRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False
    },
    "pose": {
        "type": "dict",
        "schema": PoseRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False
    },
}


class CurrentPoseRosMessageBasis(get_base_class(current_pose, current_pose_schema)):
    pass


template = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

schema = {
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


class Vector3RosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "linear": Vector3RosMessageBasis.get_template(),
    "angular": Vector3RosMessageBasis.get_template()
}

schema = {
    "linear": {
        "type": "dict",
        "schema": Vector3RosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
    "angular": {
        "type": "dict",
        "schema": Vector3RosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class TwistRosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "header": HeaderRosMessageBasis.get_template(),
    "twist": TwistRosMessageBasis.get_template()
}

schema = {
    "header": {
        "type": "dict",
        "schema": HeaderRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
    "twist": {
        "type": "dict",
        "schema": TwistRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class TwistStampedRosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "dist": 0.0,
    "dir": 0.0,
    "apara": 0.0,
    "r": 0.0,
    "slope": 0.0,
    "cant": 0.0,
    "lw": 0.0,
    "rw": 0.0
}

schema = {
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
    "rw":{
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class DtlaneRosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "aid": 0,
    "lanechange_state": 0,
    "steering_state": 0,
    "accel_state": 0,
    "stopline_state": 0,
    "event_state": 0
}

schema = {
    "aid": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "NULLSTATE": {
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
    "STR_LEFT": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "STR_RIGHT": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "STR_STRAIGHT": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "accel_state": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "stopline_state": {
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


class WaypointStateRosMessageBasis(get_base_class(template, schema)):
    pass


closest_waypoint = {
    "data": 0
}

closest_waypoint_schema = {
    "data": {
        "type": "integer",
        "required": True,
        "nullable": False,
    }
}


class ClosestWaypointRosMessageBasis(get_base_class(closest_waypoint, closest_waypoint_schema)):
    pass


dicision_maker_states = {
    "header": HeaderRosMessageBasis.get_template(),
    "main_state": "default",
    "acc_state": "default",
    "str_state": "default",
    "behavior_state": "default"
}

dicision_maker_states_schema = {
    "header": {
        "type": "dict",
        "schema": HeaderRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False
    },
    "main_state": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "acc_state": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "str_state": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "behavior_state": {
        "type": "string",
        "required": True,
        "nullable": False
    }
}


class DecisionMakerStatesRosMessageBasis(get_base_class(dicision_maker_states, dicision_maker_states_schema)):
    pass


status = {
    "route_code": "0:0_1:1",
    "time": 0.0,
    "state": "default"
}

status_schema = {
    "route_code": {
        "type": "string",
        "required": True,
        "nullable": False
    },
    "time": {
        "type": "number",
        "required": True,
        "nullable": False
    },
    "state": {
        "type": "string",
        "required": True,
        "nullable": False,
    }
}


class Status(get_base_class(status, status_schema)):
    pass



template = {
    "header": HeaderRosMessageBasis.get_template(),
    "pose": PoseRosMessageBasis.get_template()
}

schema = {
    "header": {
        "type": "dict",
        "schema": HeaderRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
    "pose": {
        "type": "dict",
        "schema": PoseRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    }
}


class PoseStampedRosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "gid": 0,
    "lid": 0,
    "pose": PoseStampedRosMessageBasis.get_template(),
    "twist": TwistStampedRosMessageBasis.get_template(),
    "dtlane": DtlaneRosMessageBasis.get_template(),
    "change_flag": 0,
    "wpstate": WaypointStateRosMessageBasis.get_template()

}

schema = {
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
        "schema": PoseStampedRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
    "twist": {
        "type": "dict",
        "schema": TwistStampedRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
    "dtlane": {
        "type": "dict",
        "schema": DtlaneRosMessageBasis.get_schema(),
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
        "schema": WaypointStateRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
}


class WaypointRosMessageBasis(get_base_class(template, schema)):
    pass


template = {
    "header": HeaderRosMessageBasis.get_template(),
    "increment": 0,
    "lane_id": 0,
    "waypoints": [WaypointRosMessageBasis.get_template()]
}

schema = {
    "header": {
        "type": "dict",
        "schema": HeaderRosMessageBasis.get_schema(),
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
        "valueschema": {
            "type": "dict",
            "schema": WaypointRosMessageBasis.get_schema(),
            "required": True,
            "nullable": False
        },
        "required": True,
        "nullable": False
    }
}


class LaneRosMessageBasis(get_base_class(template, schema)):
    pass


lane_array = {
    "lanes": [LaneRosMessageBasis.get_template()]
}

lane_array_schema = {
    "lanes": {
        "type": "list",
        "valueschema": {
            "type": "dict",
            "schema": LaneRosMessageBasis.get_schema(),
            "required": True,
            "nullable": False
        },
        "required": True,
        "nullable": False
    }
}


class LaneArrayRosMessageBasis(get_base_class(lane_array, lane_array_schema)):
    pass


state_command = {
    "data": 14
}

state_command_schema = {
    "data": {
        "type": "integer",
        "required": True,
        "nullable": False
    }
}


class StateCommandRosMessageBasis(get_base_class(state_command, state_command_schema)):
    pass


light_color = {
    "header": HeaderRosMessageBasis.get_template(),
    "traffic_light": 0
}

light_color_schema = {
    "header": {
        "type": "dict",
        "schema": HeaderRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False
    },
    "traffic_light": {
        "type": "integer",
        "required": True,
        "nullable": False
    }
}


class LightColorRosMessageBasis(get_base_class(light_color, light_color_schema)):
    pass


template = {
    "pose": PoseRosMessageBasis.get_template(),
    "convariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}

schema = {
    "position": {
        "type": "dict",
        "schema": PoseRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False,
    },
    "convariance": {
        "type": "list",
        "valueschema": {
            "type": "number",
            "required": True,
            "nullable": False
        },
        "required": True,
        "nullable": False,
    }
}


class PoseWithCovariance(get_base_class(template, schema)):
    pass


initial_pose = {
    "header": HeaderRosMessageBasis.get_template(),
    "pose": PoseWithCovariance.get_template()
}

initial_pose_schema = {
    "header": {
        "type": "dict",
        "schema": HeaderRosMessageBasis.get_schema(),
        "required": True,
        "nullable": False
    },
    "pose": {
        "type": "dict",
        "schema": PoseWithCovariance.get_schema(),
        "required": True,
        "nullable": False
    },
}


class InitialPoseRosMessageBasis(get_base_class(initial_pose, initial_pose_schema)):
    pass
