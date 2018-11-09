#!/usr/bin/env python
# coding: utf-8

import json
import math

from ams.helpers import Position, Vector, Rpy, Location
from ams.helpers import Lane as Helper
from ams.structures import LANE, Orientation, Pose


class Lane(object):

    CONST = LANE
    Helper = Helper

    def __init__(self, waypoint=None):
        self.waypoint = waypoint
        self.__lanes = None
        self.__to_lanes = None
        self.__from_lanes = None

    def load(self, path):
        self.__lanes, self.__to_lanes, self.__from_lanes = Helper.load(path)
        return True

    def set_waypoint(self, waypoint):
        self.waypoint = waypoint

    def get_lane_codes(self):
        return Helper.get_lane_codes(self.__lanes)

    def get_lane(self, lane_code):
        return Helper.get_lane(lane_code, self.__lanes)

    def get_lanes(self):
        return self.__lanes

    def get_to_lanes(self):
        return self.__to_lanes

    def get_from_lanes(self):
        return self.__from_lanes

    def get_waypoint_ids(self, lane_code):
        return Helper.get_waypoint_ids(lane_code, self.__lanes)

    def get_length(self, lane_code):
        return Helper.get_length(lane_code, self.__lanes)

    def get_yaw(self, lane_code, waypoint_id):
        return Helper.get_yaw(lane_code, waypoint_id, self.__lanes, self.waypoint.get_waypoints())

    def get_orientation(self, lane_code, waypoint_id):
        return Helper.get_orientation(lane_code, waypoint_id, self.__lanes, self.waypoint.get_waypoints())

    def get_pose(self, lane_code, waypoint_id):
        return Helper.get_pose(lane_code, waypoint_id, self.__lanes, self.waypoint.get_waypoints())

    def get_heading(self, lane_code, waypoint_id):
        return Helper.get_heading(lane_code, waypoint_id, self.__lanes, self.waypoint.get_waypoints())

    get_point_to_edge = Helper.get_point_to_edge

    def get_point_to_waypoints(self, position, waypoint_ids):
        return Helper.get_point_to_waypoints(position, waypoint_ids, self.waypoint.get_waypoints())

    def get_point_to_lane(self, position, lane_code):
        return Helper.get_point_to_waypoints(
            position, self.get_waypoint_ids(lane_code), self.__lanes, self.waypoint.get_waypoints())

    def get_point_to_lanes(self, position, lane_codes=None):
        Helper.get_point_to_lanes(position, self.__lanes, self.waypoint.get_waypoints(), lane_codes)

    def get_lane_codes_to_lanes(self, lane_codes):
        return Helper.filter_by_lane_code(self.__lanes, lane_codes)

    def get_lane_codes_from_waypoint_id(self, waypoint_id):
        return Helper.get_lane_codes_by_waypoint_id(waypoint_id, self.__lanes)

    def get_locations_by_waypoint_id(self, waypoint_id):
        return Helper.get_locations(waypoint_id, self.__lanes, self.waypoint.get_waypoints())

    def get_lane_codes_set_from_waypoint_ids(self, waypoint_ids):
        return Helper.get_lane_codes_set_by_waypoint_ids(waypoint_ids, self.__lanes)
