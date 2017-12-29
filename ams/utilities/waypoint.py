#!/usr/bin/env python
# coding: utf-8
import json


class Waypoint(object):
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
        self.__waypoints = waypoints

    def get_waypoints(self):
        return self.__waypoints

    def get_waypoint_ids(self):
        return list(self.__waypoints.keys())

    def get_latlng(self, waypoint_id):
        return self.__waypoints[waypoint_id]["lat"], self.__waypoints[waypoint_id]["lng"]

    def get_waypoint(self, waypoint_id):
        return self.__waypoints[waypoint_id]
