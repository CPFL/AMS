#!/usr/bin/env python
# coding: utf-8

from sys import float_info

from ams import get_structure_superclass, get_namedtuple_from_dict
from ams.structures import Pose


ROUTE = get_namedtuple_from_dict("CONST", {
    "DELIMITERS": {
        "WAYPOINT_ON_LANE": ":",
        "FOREWARD": ">",
        "BACKWARD": "<",
    },
    "COST_LIMIT": float_info.max
})

route_template = {
    "waypoint_ids": ["0", "1"],
    "lane_codes": ["0_1"],
    "delimiters": [":", ">", ":"]
}

'''
route_code = "[start_waypoint_id]:[joined_lane_codes]:[goal_waypoint_id]"
routes = {
    route_code: route,
}
'''

route_schema = {
    "waypoint_ids": {
        "type": "list",
        "schema": {
            "type": "string",
            "nullable": False
        },
        "required": False,
        "nullable": True,
        "minlength": 2
    },
    "lane_codes": {
        "type": "list",
        "schema": {
            "type": "string",
            "nullable": False
        },
        "required": True,
        "nullable": False,
        "minlength": 1
    },
    "delimiters": {
        "type": "list",
        "schema": {
            "type": "string",
            "nullable": False
        },
        "required": False,
        "nullable": False,
        "minlength": 1
    }
}


class Route(get_structure_superclass(route_template, route_schema)):
    pass


routes_template = [Route.get_template()]

routes_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": Route.get_schema(),
        "required": True,
        "nullable": False,
    },
    "minlength": 1
}


class Routes(get_structure_superclass(routes_template, routes_schema)):
    pass


route_detail_template = [
    {
        "waypoint_id": "0",
        "lane_code": "0_1",
        "pose": Pose.get_template(),
        "geohash": "123456789012345",
        "speed_limit": 5.5
    }
]

route_detail_schema = {
    "type": "list",
    "schema": {
        "type": "dict",
        "schema": {
            "waypoint_id": {
                "type": "string",
                "required": True,
                "nullable": False,
            },
            "lane_code": {
                "type": "string",
                "required": True,
                "nullable": False,
            },
            "pose": {
                "type": "dict",
                "schema": Pose.get_schema(),
                "required": True,
                "nullable": False
            },
            "geohash": {
                "type": "string",
                "required": True,
                "nullable": False,
            },
            "speed_limit": {
                "type": "number",
                "required": True,
                "nullable": False,
            }
        }
    },
    "minlength": 1
}


class RouteDetail(get_structure_superclass(route_detail_template, route_detail_schema)):
    Pose = Pose


route_point_template = {
    "route_code": "0:0>1:1",
    "index": 0
}

route_point_schema = {
    "route_code": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "index": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
}


class RoutePoint(get_structure_superclass(route_point_template, route_point_schema)):
    pass


route_section_template = {
    "route_code": "0:0>1:1",
    "start_index": 0,
    "end_index": 1
}

route_section_schema = {
    "route_code": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "start_index": {
        "type": "integer",
        "required": True,
        "nullable": False,
    },
    "end_index": {
        "type": "integer",
        "required": True,
        "nullable": False,
    }
}


class RouteSection(get_structure_superclass(route_section_template, route_section_schema)):
    pass
