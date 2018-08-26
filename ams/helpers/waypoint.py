#!/usr/bin/env python
# coding: utf-8

import json
import geohash
from copy import copy

from ams.helpers import Position, Rpy
from ams.structures import WAYPOINT, Pose, Orientation


class Waypoint(object):

    CONST = WAYPOINT

    @classmethod
    def load(cls, path):
        with open(path, "r") as f:
            data = json.load(f)
            waypoints = dict(map(
                lambda x: (x[0], {
                    "waypoint_id": x[1]["waypointID"],
                    "geohash": geohash.encode(
                        float(x[1]["lat"]), float(x[1]["lng"]), precision=WAYPOINT.GEOHASH_PRECISION),
                    "position": Position.new_position(*map(lambda key: x[1][key], ("x", "y", "z"))),
                    "yaw": x[1]["yaw"],
                    "speed_limit": x[1]["speedLimit"]
                }),
                data["waypoints"].items()))
        return waypoints

    @classmethod
    def get_waypoint_ids(cls, waypoints):
        return tuple(waypoints.keys())

    @classmethod
    def get_latlng(cls, waypoint_id, waypoints):
        return geohash.decode(waypoints[waypoint_id]["geohash"])

    @classmethod
    def get_geohash(cls, waypoint_id, waypoints):
        return waypoints[waypoint_id]["geohash"]

    @classmethod
    def get_position(cls, waypoint_id, waypoints):
        return copy(waypoints[waypoint_id]["position"])

    @classmethod
    def get_yaw(cls, waypoint_id, waypoints):
        return waypoints[waypoint_id]["yaw"]

    @classmethod
    def get_velocity(cls, waypoint_id, waypoints):
        return waypoints[waypoint_id]["speed_limit"]

    @classmethod
    def get_orientation(cls, waypoint_id, waypoints):
        return Orientation.new_data(
            quaternion=dict(zip(
                ["w", "x", "y", "z"],
                Rpy.to_quaternion([0, 0, 1], cls.get_yaw(waypoint_id, waypoints)))),
            rpy=Rpy.Structure.new_data(
                roll=None,
                pitch=None,
                yaw=cls.get_yaw(waypoint_id, waypoints)
            )
        )

    @classmethod
    def get_pose(cls, waypoint_id, waypoints):
        return Pose.new_data(
            position=cls.get_position(waypoint_id, waypoints),
            orientation=cls.get_orientation(waypoint_id, waypoints)
        )
