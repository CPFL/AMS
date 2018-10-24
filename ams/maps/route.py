#!/usr/bin/env python
# coding: utf-8

from ams import logger
from ams.helpers import Location
from ams.helpers import Route as Helper
from ams.structures import ROUTE, RouteDetail


class Route(object):

    CONST = ROUTE
    Helper = Helper

    def __init__(self):
        self.__getRouteCost = Helper.get_length
        self.__waypoint = None
        self.__arrow = None
        self.__routes = None

    def set_waypoint(self, waypoint):
        self.__waypoint = waypoint

    def set_arrow(self, arrow):
        self.__arrow = arrow

    def set_cost_function(self, cost_function):
        self.__getRouteCost = cost_function

    load = Helper.load

    def get_route_ids(self):
        return Helper.get_route_ids(self.__routes)

    def get_route(self, route_id):
        return Helper.get_route(route_id, self.__routes)

    def get_detail(self, route):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.waypoint_ids[0]
        goal_waypoint_id = route.waypoint_ids[-1]
        details = []
        for i, arrow_code in enumerate(arrow_codes):
            waypoint_ids = self.__arrow.get_waypoint_ids(arrow_code)
            js = 0
            if i == 0 and start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if i == len(arrow_codes)-1 and goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1
            for j in range(js+1, je):
                details.append({
                    "waypoint_id": waypoint_ids[j - 1],
                    "arrow_code": arrow_code,
                    "pose": self.__arrow.get_pose(arrow_code, waypoint_ids[j - 1]),
                    "geohash": self.__waypoint.get_geohash(waypoint_ids[j - 1]),
                    "speed_limit": self.__waypoint.get_speed_limit(waypoint_ids[j - 1])
                })
            if arrow_code == arrow_codes[-1]:
                details.append({
                    "waypoint_id": waypoint_ids[je - 1],
                    "arrow_code": arrow_code,
                    "pose": self.__arrow.get_pose(arrow_code, waypoint_ids[je - 1]),
                    "geohash": self.__waypoint.get_geohash(waypoint_ids[je - 1]),
                    "speed_limit": self.__waypoint.get_speed_limit(waypoint_ids[je - 1])
                })
        return RouteDetail.new_data(details)

    def get_locations(self, route):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.waypoint_ids[0]
        goal_waypoint_id = route.waypoint_ids[-1]
        locations = []
        for i, arrow_code in enumerate(arrow_codes):
            waypoint_ids = self.__arrow.get_waypoint_ids(arrow_code)
            js = 0
            if i == 0 and start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if i == len(arrow_codes)-1 and goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1
            for j in range(js+1, je):
                locations.append({
                    "waypoint_id": waypoint_ids[j - 1],
                    "arrow_code": arrow_code,
                    "geohash": self.__waypoint.get_geohash(waypoint_ids[j - 1])
                })
            if arrow_code == arrow_codes[-1]:
                locations.append({
                    "waypoint_id": waypoint_ids[je - 1],
                    "arrow_code": arrow_code,
                    "geohash": self.__waypoint.get_geohash(waypoint_ids[je - 1])
                })

        return Location.new_locations(locations)

    def get_waypoint_ids(self, route_code):
        return Helper.get_waypoint_ids(route_code, self.__arrow.get_arrows())

    def get_route_length(self, route):
        return Helper.get_length(route, self.__arrow.get_arrows(), self.__waypoint.get_waypoints())

    def get_distance_of_waypoints(self, waypoint_ids):
        distance = 0.0
        for i in range(1, len(waypoint_ids)):
            distance += self.__arrow.get_distance(
                self.__waypoint.get_position(waypoint_ids[i]), self.__waypoint.get_position(waypoint_ids[i - 1]))
        return distance

    def get_sliced_route(self, route, length):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.waypoint_ids[0]
        goal_waypoint_id = route.waypoint_ids[-1]
        total_length = 0.0
        sliced_goal_waypoint_id = start_waypoint_id
        sliced_arrow_codes = [arrow_codes[0]]
        for arrow_code in arrow_codes[1:]:
            waypoint_ids = self.__arrow.get_waypoint_ids(arrow_code)
            js = 0
            if start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1

            for i in range(js+1, je):
                total_length += self.__arrow.get_distance(
                    self.__waypoint.get_position(
                        waypoint_ids[i]), self.__waypoint.get_position(waypoint_ids[i-1]))
                if length <= total_length:
                    return Helper.new_route([start_waypoint_id, sliced_goal_waypoint_id], sliced_arrow_codes)
                if len(sliced_arrow_codes) == 0 or sliced_arrow_codes[-1] != arrow_code:
                    sliced_arrow_codes.append(arrow_code)
                sliced_goal_waypoint_id = waypoint_ids[i]
        return route

    def get_shortest_routes(self, start, goals, cost_limit=ROUTE.COST_LIMIT, reverse=False):
        return Helper.get_shortest_routes(
            start, goals, self.__arrows, self.__to_arrows, self.__from_arrows, self.__waypoint.get_waypoints(),
            self.__getRouteCost, cost_limit, reverse)

    def add_length_to_routes(self, routes):
        for route_id in routes:
            routes[route_id]["length"] = self.get_route_length(routes[route_id])
        return routes

    def get_speed_limits(self, route):
        waypoint_ids = self.get_route_waypoint_ids(route)
        speed_limits = list(map(self.__waypoint.get_speed_limit, waypoint_ids))
        return speed_limits

    def generate_lane_array(self, route_code):
        return Helper.generate_lane_array(route_code, self.__arrow.get_arrows(), self.__waypoint.get_waypoints())

    def get_route_point_pose_and_location(self, route_point):
        return Helper.get_route_point_pose_and_location(
            route_point, self.__arrow.get_arrows(), self.__waypoint.get_waypoints())

    def generate_arrow_code_waypoint_id_relations(self, route_code):
        return Helper.generate_arrow_code_waypoint_id_relations(route_code, self.__arrow.get_arrows())
