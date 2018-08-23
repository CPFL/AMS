#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Waypoint as Helper
from ams.structures import WAYPOINT


class Waypoint(object):

    CONST = WAYPOINT

    def __init__(self):
        self.__waypoints = {}

    def load(self, path):
        self.__waypoints = Helper.load(path)
        return True

    def get_waypoints(self):
        return self.__waypoints

    def get_waypoint_ids(self):
        return Helper.get_waypoint_ids(self.__waypoints)

    def get_latlng(self, waypoint_id):
        return Helper.get_latlng(waypoint_id, self.__waypoints)

    def get_geohash(self, waypoint_id):
        return Helper.get_geohash(waypoint_id, self.__waypoints)

    def get_position(self, waypoint_id):
        return Helper.get_position(waypoint_id, self.__waypoints)

    def get_yaw(self, waypoint_id):
        return Helper.get_yaw(waypoint_id, self.__waypoints)

    def get_orientation(self, waypoint_id):
        return Helper.get_orientation(waypoint_id, self.__waypoints)

    def get_pose(self, waypoint_id):
        return Helper.get_pose(waypoint_id, self.__waypoints)

    def get_speed_limit(self, waypoint_id):
        return Helper.get_velocity(waypoint_id, self.__waypoints)
