#!/usr/bin/env python
# coding: utf-8

import json


class Intersection(object):
    def __init__(self):
        self.__intersections = {}

    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.set_intersections(data["intersections"])
        return True

    def connect_to_redis(self, _host, _port, _dbname):
        return self

    def set_intersections(self, intersections):
        self.__intersections = intersections

    def get_intersections(self):
        return self.__intersections

    def get_intersection(self, intersection_id):
        return self.__intersections[intersection_id]

    def get_intersection_ids(self):
        return list(self.__intersections.keys())

    def get_traffic_signals(self, intersection_id):
        return self.__intersections[intersection_id]["trafficSignals"]

    def get_entry_exit_route_ids(self, intersection_id):
        return list(self.__intersections[intersection_id]["entryExitRoutes"].keys())

    def get_border_point(self, intersection_id, lane_code):
        return list(filter(
            lambda x: x["lane_code"] == lane_code,
            self.__intersections[intersection_id]["borderPoints"]))[0]

    def get_to_in_border_waypoint_ids(self, intersection_id):
        return list(map(
            lambda x: x["prevWaypointID"],
            filter(lambda x: x["toIn"], self.__intersections[intersection_id]["borderPoints"])))

    def get_to_in_lane_codes(self, intersection_id):
        return list(map(
            lambda x: x["lane_code"],
            filter(lambda x: x["toIn"], self.__intersections[intersection_id]["borderPoints"])))

    def get_entry_exit_route_lane_codes_set(self, intersection_id):
        return list(map(
            lambda x: (x[0], x[1]["lane_codes"]),
            self.__intersections[intersection_id]["entryExitRoutes"].items()))

    def get_entry_exit_route_ids_in_lane_codes(self, intersection_id, lane_codes):
        entry_exit_route_ids = []
        to_in_lane_codes = self.get_to_in_lane_codes(intersection_id)
        for i, lane_code in enumerate(lane_codes):
            if lane_code in to_in_lane_codes:
                for entry_exit_route_id, entryExitRoute in \
                        self.__intersections[intersection_id]["entryExitRoutes"].items():
                    entry_exit_route_lane_codes = entryExitRoute["lane_codes"]
                    length = len(entry_exit_route_lane_codes)
                    if lane_codes[i:i+length] == entry_exit_route_lane_codes:
                        entry_exit_route_ids.append(entry_exit_route_id)
        return entry_exit_route_ids

    def get_lane_codes_of_entry_exit_route(self, intersection_id, entry_exit_route_id):
        return self.__intersections[intersection_id]["entryExitRoutes"][entry_exit_route_id]["lane_codes"]
