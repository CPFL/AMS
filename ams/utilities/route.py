#!/usr/bin/env python
# coding: utf-8
from math import hypot
import json
from sys import float_info

from ams import Arrow

METER_PER_BL = 100749.999999999998464


class Route(object):
    DELIMITER = ":"

    def __init__(self):
        self.__getRouteCost = self.get_route_length
        self.__waypoint = None
        self.__waypoints = {}
        self.__arrow = None
        self.__arrows = {}
        self.__fixed_routes = None

    def set_waypoint(self, waypoint):
        self.__waypoint = waypoint
        self.__waypoints = waypoint.get_waypoints()

    def set_arrow(self, arrow):
        self.__arrow = arrow
        self.__arrows = arrow.get_arrows()

    def set_cost_function(self, cost_function):
        """
        cost_function(arg = route{"start_waypoint_id": X, "goal_waypoint_id": X, "arrow_codes": X})
        :param cost_function: 
        :return: 
        """
        self.__getRouteCost = cost_function

    def load(self, path):
        with open(path, "r") as f:
            self.__fixed_routes = json.load(f)
        return True

    def get_fixed_route(self, route_code):
        return self.__fixed_routes[route_code]

    def __arrow_code_to_route(self, arrow_code):
        arrow = self.__arrows[arrow_code]
        return {
            "start_waypoint_id": arrow["waypointIDs"][0],
            "goal_waypoint_id": arrow["waypointIDs"][-1],
            "arrow_codes": [arrow_code]
        }

    def get_arrow_waypoint_array(self, route):
        arrow_codes = route["arrow_codes"]
        start_waypoint_id = route["start_waypoint_id"]
        goal_waypoint_id = route["goal_waypoint_id"]
        arrow_waypoint_array = []
        for arrow_code in arrow_codes:
            waypoint_ids = self.__arrows[arrow_code]["waypointIDs"]
            js = 0
            if start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1
            for i in range(js+1, je):
                arrow_waypoint_array.append({"waypoint_id": waypoint_ids[i - 1], "arrow_code": arrow_code})
            if arrow_code == arrow_codes[-1]:
                arrow_waypoint_array.append({"waypoint_id": waypoint_ids[je - 1], "arrow_code": arrow_code})
        return arrow_waypoint_array

    def get_route_waypoint_ids(self, route):
        arrow_codes = route["arrow_codes"]
        start_waypoint_id = route["start_waypoint_id"]
        goal_waypoint_id = route["goal_waypoint_id"]
        route_waypoint_ids = []
        for arrow_code in arrow_codes:
            waypoint_ids = self.__arrows[arrow_code]["waypointIDs"]
            js = 0
            if start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1
            for i in range(js+1, je):
                route_waypoint_ids.append(waypoint_ids[i - 1])
            if arrow_code == arrow_codes[-1]:
                route_waypoint_ids.append(waypoint_ids[je - 1])
        return route_waypoint_ids

    def get_route_length(self, route):
        arrow_codes = route["arrow_codes"]
        start_waypoint_id = route["start_waypoint_id"]
        goal_waypoint_id = route["goal_waypoint_id"]
        length = 0.0
        for arrow_code in arrow_codes:
            waypoint_ids = self.__arrows[arrow_code]["waypointIDs"]
            js = 0
            if start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1

            if js == 0 and je == len(waypoint_ids):
                length += self.__arrows[arrow_code]["length"]
            else:
                for i in range(js+1, je):
                    waypoint1 = self.__waypoints[waypoint_ids[i - 1]]
                    waypoint2 = self.__waypoints[waypoint_ids[i]]
                    length += METER_PER_BL * hypot(
                        waypoint1["lat"] - waypoint2["lat"], waypoint1["lng"] - waypoint2["lng"])
        return length

    def __is_directly_reach(self, arrow_code, start_waypoint_id, goal_waypoint_id, reverse):
        waypoint_ids = self.__arrows[arrow_code]["waypointIDs"]
        is_directly_reach = waypoint_ids.index(start_waypoint_id) <= waypoint_ids.index(goal_waypoint_id)
        return is_directly_reach if not reverse else not is_directly_reach

    def get_distance_of_waypoints(self, waypoint_ids):
        distance = 0.0
        for i in range(1, len(waypoint_ids)):
            distance += METER_PER_BL * hypot(
                self.__waypoints[waypoint_ids[i]]["lat"] - self.__waypoints[waypoint_ids[i - 1]]["lat"],
                self.__waypoints[waypoint_ids[i]]["lng"] - self.__waypoints[waypoint_ids[i - 1]]["lng"]
            )
        return distance

    def get_sliced_route(self, route, length):
        arrow_codes = route["arrow_codes"]
        start_waypoint_id = route["start_waypoint_id"]
        goal_waypoint_id = route["goal_waypoint_id"]
        total_length = 0.0
        sliced_goal_waypoint_id = start_waypoint_id
        sliced_arrow_codes = []
        for arrow_code in arrow_codes:
            waypoint_ids = self.__arrows[arrow_code]["waypointIDs"]
            js = 0
            if start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1

            for i in range(js+1, je):
                total_length += METER_PER_BL * hypot(
                    self.__waypoints[waypoint_ids[i]]["lat"]-self.__waypoints[waypoint_ids[i-1]]["lat"],
                    self.__waypoints[waypoint_ids[i]]["lng"]-self.__waypoints[waypoint_ids[i-1]]["lng"])
                if length <= total_length:
                    return {
                        "start_waypoint_id": start_waypoint_id,
                        "goal_waypoint_id": sliced_goal_waypoint_id,
                        "arrow_codes": sliced_arrow_codes
                    }
                if len(sliced_arrow_codes) == 0 or sliced_arrow_codes[-1] != arrow_code:
                    sliced_arrow_codes.append(arrow_code)
                sliced_goal_waypoint_id = waypoint_ids[i]

        return route

    def get_waypoint_ids_on_route(self, route, limit_length=None):
        waypoint_ids = []
        if limit_length is None:
            return self.get_route_waypoint_ids(route)
        else:
            total_length = 0.0
            route_waypoint_ids = self.get_route_waypoint_ids(route)
            for i in range(1, len(route_waypoint_ids)):
                waypoint_ids.append(route_waypoint_ids[i-1])
                total_length += METER_PER_BL * hypot(
                    self.__waypoints[route_waypoint_ids[i]]["lat"]-self.__waypoints[route_waypoint_ids[i-1]]["lat"],
                    self.__waypoints[route_waypoint_ids[i]]["lng"]-self.__waypoints[route_waypoint_ids[i-1]]["lng"])
                if limit_length < total_length:
                    return waypoint_ids
        return waypoint_ids

    def get_shortest_routes(self, start, goals, cost_limit=float_info.max, reverse=False):
        # todo: コストと距離の分離
        """
        各目的地ごとの最短距離ルートを計算
        # Dijkstra's algorithm
        """
        next_arrows = self.__arrow.get_to_arrows()
        waypoint_id_end = self.__arrows[start["arrow_code"]]["waypointIDs"][-1]
        local_start_waypoint_id = start["waypoint_id"]
        local_goal_waypoint_id = waypoint_id_end
        if reverse:
            next_arrows = self.__arrow.get_from_arrows()
            waypoint_id_end = self.__arrows[start["arrow_code"]]["waypointIDs"][0]
            local_start_waypoint_id = waypoint_id_end
            local_goal_waypoint_id = start["waypoint_id"]

        cost_start_to_end = self.__getRouteCost({
            "start_waypoint_id": local_start_waypoint_id,
            "goal_waypoint_id": local_goal_waypoint_id,
            "arrow_codes": [start["arrow_code"]]
        })

        goal_arrow_candidates = self.__get_goal_arrow_candidates(goals, reverse)

        checked_arrow_code = []
        current_arrow_code = start["arrow_code"]

        end_arrows = {current_arrow_code: {"cost": cost_start_to_end, "prevArrows": []}}
        shortest_routes = {}

        # check same arrow
        for goal_points in goal_arrow_candidates.values():
            for goal_candidate in goal_points.values():
                if start["arrow_code"] == goal_candidate["arrow_code"]:
                    if self.__is_directly_reach(
                            start["arrow_code"], start["waypoint_id"], goal_candidate["waypoint_id"], reverse):
                        print("both start and goal on same arrow. and reach directly")
                        local_start_waypoint_id = start["waypoint_id"]
                        local_goal_waypoint_id = goal_candidate["waypoint_id"]
                        if reverse:
                            local_start_waypoint_id = goal_candidate["waypoint_id"]
                            local_goal_waypoint_id = start["waypoint_id"]
                        cost_start_to_goal = self.__getRouteCost({
                            "start_waypoint_id": local_start_waypoint_id,
                            "goal_waypoint_id": local_goal_waypoint_id,
                            "arrow_codes": [start["arrow_code"]]
                        })
                        shortest_routes[goal_candidate["goal_id"]] = {
                            "goal_id": goal_candidate["goal_id"],
                            "start_waypoint_id": start["waypoint_id"],
                            "goal_waypoint_id": goal_candidate["waypoint_id"],
                            "cost": cost_start_to_goal,
                            "arrow_codes": [start["arrow_code"]],
                        }

        while True:
            next_arrow_codes = next_arrows[current_arrow_code]

            if 0 == len(next_arrow_codes):
                end_arrows[current_arrow_code]["cost"] = float_info.max

                end_arrows_filtered = dict(filter(lambda x: x[1]["cost"] < cost_limit, end_arrows.items()))
                if len(end_arrows_filtered) == 0:
                    break
                current_arrow_code = min(end_arrows_filtered.items(), key=lambda x: x[1]["cost"])[0]

                continue

            for nextArrowID in next_arrow_codes:
                if nextArrowID in checked_arrow_code and nextArrowID != start["arrow_code"]:
                    continue
                if nextArrowID in end_arrows:
                    continue

                # update end_arrows
                end_arrows[nextArrowID] = {
                    "cost": end_arrows[current_arrow_code]["cost"] + self.__getRouteCost(
                        self.__arrow_code_to_route(current_arrow_code)),
                    "prevArrows": [current_arrow_code] + end_arrows[current_arrow_code]["prevArrows"]
                }

                for goal_candidate in goal_arrow_candidates.get(nextArrowID, {}).values():
                    if end_arrows[nextArrowID]["cost"] + goal_candidate["cost"] < cost_limit:
                        shortest_route = {
                            "goal_id": goal_candidate["goal_id"],
                            "start_waypoint_id": start["waypoint_id"],
                            "goal_waypoint_id": goal_candidate["waypoint_id"],
                            "cost": end_arrows[nextArrowID]["cost"] + goal_candidate["cost"],
                            "arrow_codes": [nextArrowID] + end_arrows[nextArrowID]["prevArrows"],
                        }
                        shortest_routes[goal_candidate["goal_id"]] = shortest_route

            end_arrows.pop(current_arrow_code)
            checked_arrow_code.append(current_arrow_code)

            if len(shortest_routes) == len(goal_arrow_candidates):
                break

            end_arrows_filtered = dict(filter(lambda x: x[1]["cost"] < cost_limit, end_arrows.items()))
            if len(end_arrows_filtered) == 0:
                break
            current_arrow_code = min(end_arrows_filtered.items(), key=lambda x: x[1]["cost"])[0]

        # reverse arrow_codes
        if not reverse:
            for routeID in shortest_routes:
                shortest_routes[routeID]["arrow_codes"].reverse()

        return shortest_routes

    def add_length_to_routes(self, routes):
        for routeID in routes:
            routes[routeID]["length"] = self.get_route_length(routes[routeID])
        return routes

    def __get_goal_arrow_candidates(self, goals, reverse):
        goal_arrow_candidates = {}
        for goal in goals:
            goal_id = goal["goal_id"]
            arrow_code = goal["arrow_code"]
            if reverse:
                end_waypoint_id = self.__arrows[arrow_code]["waypointIDs"][-1]
            else:
                end_waypoint_id = self.__arrows[arrow_code]["waypointIDs"][0]
            cost = self.__getRouteCost({
                "start_waypoint_id": end_waypoint_id,
                "goal_waypoint_id": goal["waypoint_id"],
                "arrow_codes": [arrow_code]
            })

            goal_points = goal_arrow_candidates.get(arrow_code, {})
            goal_points[goal_id] = {
                "arrow_code": arrow_code,
                "goal_id": goal_id,
                "waypoint_id": goal["waypoint_id"],
                "cost": cost,
            }
            goal_arrow_candidates[arrow_code] = goal_points

        return goal_arrow_candidates

    @staticmethod
    def get_route_code(route):
        joined_arrow_codes = Arrow.DELIMITER.join(list(map(
            lambda x: x.split(Arrow.DELIMITER)[0],
            route["arrow_codes"])) + [route["arrow_codes"][-1].split(Arrow.DELIMITER)[-1]])
        route_code = Route.DELIMITER.join(map(
            str, [route["start_waypoint_id"], joined_arrow_codes, route["goal_waypoint_id"]]))
        return route_code

    @staticmethod
    def split_route_code(route_code):
        start_waypoint_id, joined_arrow_codes, goal_waypoint_id = route_code.split(Route.DELIMITER)
        waypoint_ids = joined_arrow_codes.split(Arrow.DELIMITER)
        arrow_codes = []
        for i in range(1, len(waypoint_ids)):
            arrow_codes.append(Arrow.DELIMITER.join(waypoint_ids[i-1:i+1]))
        return start_waypoint_id, arrow_codes, goal_waypoint_id
