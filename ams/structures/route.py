#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass, get_namedtuple_from_dict
from ams.structures import Pose


ROUTE = get_namedtuple_from_dict("CONST", {
    "DELIMITER": ":"
})

route_template = {
    "start_waypoint_id": "0",
    "goal_waypoint_id": "1",
    "arrow_codes": ["0_1"]
}

'''
route_code = "[start_waypoint_id]:[joined_arrow_codes]:[goal_waypoint_id]"
routes = {
    route_code: route,
}
'''

route_schema = {
    "start_waypoint_id": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "goal_waypoint_id": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "arrow_codes": {
        "type": "list",
        "schema": {
            "type": "string",
            "nullable": False
        },
        "required": True,
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
        "arrow_code": "0_1",
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
            "arrow_code": {
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
