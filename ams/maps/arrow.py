#!/usr/bin/env python
# coding: utf-8

import json
import math

from ams.helpers import Position, Vector, Rpy, Location
from ams.helpers import Arrow as Helper
from ams.structures import ARROW, Orientation, Pose


class Arrow(object):

    CONST = ARROW
    Helper = Helper

    def __init__(self, waypoint=None):
        self.waypoint = waypoint
        self.__arrows = None
        self.__to_arrows = None
        self.__from_arrows = None

    def load(self, path):
        self.__arrows, self.__to_arrows, self.__from_arrows = Helper.load(path)
        return True

    def set_waypoint(self, waypoint):
        self.waypoint = waypoint

    def get_arrow_codes(self):
        return Helper.get_arrow_codes(self.__arrows)

    def get_arrow(self, arrow_code):
        return Helper.get_arrow(arrow_code, self.__arrows)

    def get_arrows(self):
        return self.__arrows

    def get_to_arrows(self):
        return self.__to_arrows

    def get_from_arrows(self):
        return self.__from_arrows

    def get_waypoint_ids(self, arrow_code):
        return Helper.get_waypoint_ids(arrow_code, self.__arrows)

    def get_length(self, arrow_code):
        return Helper.get_length(arrow_code, self.__arrows)

    def get_yaw(self, arrow_code, waypoint_id):
        return Helper.get_yaw(arrow_code, waypoint_id, self.__arrows, self.waypoint.get_waypoints())

    def get_orientation(self, arrow_code, waypoint_id):
        return Helper.get_orientation(arrow_code, waypoint_id, self.__arrows, self.waypoint.get_waypoints())

    def get_pose(self, arrow_code, waypoint_id):
        return Helper.get_pose(arrow_code, waypoint_id, self.__arrows, self.waypoint.get_waypoints())

    def get_heading(self, arrow_code, waypoint_id):
        return Helper.get_heading(arrow_code, waypoint_id, self.__arrows, self.waypoint.get_waypoints())

    get_point_to_edge = Helper.get_point_to_edge

    def get_point_to_waypoints(self, position, waypoint_ids):
        return Helper.get_point_to_waypoints(position, waypoint_ids, self.waypoint.get_waypoints())

    def get_point_to_arrow(self, position, arrow_code):
        return Helper.get_point_to_waypoints(
            position, self.get_waypoint_ids(arrow_code), self.__arrows, self.waypoint.get_waypoints())

    def get_point_to_arrows(self, position, arrow_codes=None):
        Helper.get_point_to_arrows(position, self.__arrows, self.waypoint.get_waypoints(), arrow_codes)

    def get_arrow_codes_to_arrows(self, arrow_codes):
        return Helper.filter_by_arrow_code(self.__arrows, arrow_codes)

    def get_arrow_codes_from_waypoint_id(self, waypoint_id):
        return Helper.get_arrow_codes_by_waypoint_id(waypoint_id, self.__arrows)

    def get_locations_by_waypoint_id(self, waypoint_id):
        return Helper.get_locations(waypoint_id, self.__arrows, self.waypoint.get_waypoints())

    def get_arrow_codes_set_from_waypoint_ids(self, waypoint_ids):
        return Helper.get_arrow_codes_set_by_waypoint_ids(waypoint_ids, self.__arrows)
