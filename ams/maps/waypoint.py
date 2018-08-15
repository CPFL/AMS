#!/usr/bin/env python
# coding: utf-8

import json
import geohash

from ams.helpers import Position, Rpy
from ams.structures import WAYPOINT, Pose, Orientation


class Waypoint(object):

    CONST = WAYPOINT

    def __init__(self):
        self.__waypoints = {}

    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.set_waypoints(data["waypoints"])
        return True

    def connect_to_redis(self, _host, _port, _dbname):
        return self

    def set_waypoints(self, waypoints):
        self.__waypoints = dict(map(
            lambda x: (x[0], {
                "waypoint_id": x[1]["waypointID"],
                "geohash": geohash.encode(
                    float(x[1]["lat"]), float(x[1]["lng"]), precision=WAYPOINT.GEOHASH_PRECISION),
                "position": Position.new_position(*map(lambda key: x[1][key], ("x", "y", "z"))),
                "yaw": x[1]["yaw"],
                "speed_limit": x[1]["speedLimit"]
            }),
            waypoints.items()))

    def get_waypoints(self):
        return self.__waypoints

    def get_waypoint_ids(self):
        return list(self.__waypoints.keys())

    def get_latlng(self, waypoint_id):
        return geohash.decode(self.__waypoints[waypoint_id]["geohash"])

    def get_geohash(self, waypoint_id):
        return self.__waypoints[waypoint_id]["geohash"]

    def get_position(self, waypoint_id):
        return self.__waypoints[waypoint_id]["position"]

    def get_yaw(self, waypoint_id):
        return self.__waypoints[waypoint_id]["yaw"]

    def get_orientation(self, waypoint_id):
        return Orientation.new_data(
            quaternion=dict(zip(
                ["w", "x", "y", "z"],
                Rpy.to_quaternion([0, 0, 1], self.get_yaw(waypoint_id)))),
            rpy=Rpy.Structure.new_data(
                roll=None,
                pitch=None,
                yaw=self.get_yaw(waypoint_id)
            )
        )

    def get_pose(self, waypoint_id):
        return Pose.new_data(
            position=self.get_position(waypoint_id),
            orientation=self.get_orientation(waypoint_id)
        )

    def get_speed_limit(self, waypoint_id):
        return self.__waypoints[waypoint_id]["speed_limit"]
