#!/usr/bin/env python
# coding: utf-8

import json
import Geohash
from transforms3d.quaternions import axangle2quat

from ams import Position
from ams.structures import Pose, Rpy, Orientation


class Waypoint(object):
    GEOHASH_PRECISION = 15

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
                "geohash": Geohash.encode(
                    float(x[1]["lat"]), float(x[1]["lng"]), precision=Waypoint.GEOHASH_PRECISION),
                "np_position": Position.new_np_position(x[1]["x"], x[1]["y"], x[1]["z"]),
                "yaw": x[1]["yaw"],
                "speed_limit": x[1]["speedLimit"]
            }),
            waypoints.items()))

    def get_waypoint_ids(self):
        return list(self.__waypoints.keys())

    def get_latlng(self, waypoint_id):
        return Geohash.decode(self.__waypoints[waypoint_id]["geohash"])

    def get_geohash(self, waypoint_id):
        return self.__waypoints[waypoint_id]["geohash"]

    def get_np_position(self, waypoint_id):
        return self.__waypoints[waypoint_id]["np_position"]

    def get_position(self, waypoint_id):
        return Position.new_position_from_np_position(self.get_np_position(waypoint_id))

    def get_xyz(self, waypoint_id):
        return self.__waypoints[waypoint_id]["np_position"].data[:]

    def get_yaw(self, waypoint_id):
        return self.__waypoints[waypoint_id]["yaw"]

    def get_orientation(self, waypoint_id):
        return Orientation.new_data(
            quaternion=dict(zip(
                ["w", "x", "y", "z"],
                axangle2quat([0, 0, 1], self.get_yaw(waypoint_id)))),
            rpy=Rpy.new_data(
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
