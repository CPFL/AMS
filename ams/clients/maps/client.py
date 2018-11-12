#!/usr/bin/env python
# coding: utf-8

from ams.maps import Waypoint, Lane, Route, MapMatch


class MapsClient(object):

    Waypoint = Waypoint
    Lane = Lane
    Route = Route
    MapMatch = MapMatch

    def __init__(self):
        self.waypoint = Waypoint()

        self.lane = Lane()
        self.lane.set_waypoint(self.waypoint)

        self.route = Route()
        self.route.set_waypoint(self.waypoint)
        self.route.set_lane(self.lane)

        self.map_match = MapMatch()
        self.map_match.set_waypoint(self.waypoint)
        self.map_match.set_lane(self.lane)
        self.map_match.set_route(self.route)

    def connect(self):
        pass

    def load_waypoint_json_file(self, file_path):
        self.waypoint.load(file_path)

    def load_lane_json_file(self, file_path):
        self.lane.load(file_path)

    def disconnect(self):
        pass
