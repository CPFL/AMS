#!/usr/bin/env python
# coding: utf-8

import json
from sys import float_info
import numpy as np

from ams import Arrow, Location
from ams.structures import ROUTE
from ams.structures import Route as Structure
from ams.structures import Routes as Structures

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class Route(object):

    CONST = ROUTE

    def __init__(self):
        self.__getRouteCost = self.get_route_length
        self.__waypoint = None
        self.__arrow = None
        self.__routes = None

    def set_waypoint(self, waypoint):
        self.__waypoint = waypoint

    def set_arrow(self, arrow):
        self.__arrow = arrow

    @staticmethod
    def new_route(start_waypoint_id, goal_waypoint_id, arrow_codes):
        # if start_waypoint_id == goal_waypoint_id:
        #     print("USE Route.new_point_route().")
        return Structure.new_data(
            start_waypoint_id=start_waypoint_id,
            goal_waypoint_id=goal_waypoint_id,
            arrow_codes=arrow_codes
        )

    validate_route = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def new_point_route(waypoint_id, arrow_code):
        return Structure.new_data(
            start_waypoint_id=waypoint_id,
            goal_waypoint_id=waypoint_id,
            arrow_codes=[arrow_code]
        )

    @staticmethod
    def new_routes(routes):
        return Structures.new_data(routes)

    def set_cost_function(self, cost_function):
        self.__getRouteCost = cost_function

    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.__routes = dict(map(lambda x: (x["ID"], x), data["routes"]))
        return True

    def get_route_ids(self):
        return self.__routes.keys()

    def get_route(self, route_id):
        start_waypoint_id, arrow_codes, goal_waypoint_id = \
            self.split_route_code(self.__routes[route_id]["code"])
        return Route.new_route(
            start_waypoint_id=start_waypoint_id,
            goal_waypoint_id=goal_waypoint_id,
            arrow_codes=arrow_codes)

    def __arrow_code_to_route(self, arrow_code):
        arrow = self.__arrow.get_arrow(arrow_code)
        return self.new_route(
            arrow["waypointIDs"][0], arrow["waypointIDs"][-1], [arrow_code])

    def get_locations(self, route):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.start_waypoint_id
        goal_waypoint_id = route.goal_waypoint_id
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
                locations.append({"waypoint_id": waypoint_ids[j - 1], "arrow_code": arrow_code})
            if arrow_code == arrow_codes[-1]:
                locations.append({"waypoint_id": waypoint_ids[je - 1], "arrow_code": arrow_code})

        return Location.new_locations(locations)

    def get_arrow_waypoint_array(self, route):
        # print("use get_locations()")
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.start_waypoint_id
        goal_waypoint_id = route.goal_waypoint_id
        arrow_waypoint_array = []
        for i, arrow_code in enumerate(arrow_codes):
            waypoint_ids = self.__arrow.get_waypoint_ids(arrow_code)
            js = 0
            if i == 0 and start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if i == len(arrow_codes)-1 and goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1
            for j in range(js+1, je):
                arrow_waypoint_array.append({"waypoint_id": waypoint_ids[j - 1], "arrow_code": arrow_code})
            if arrow_code == arrow_codes[-1]:
                arrow_waypoint_array.append({"waypoint_id": waypoint_ids[je - 1], "arrow_code": arrow_code})
        return arrow_waypoint_array

    @staticmethod
    def get_route_node_arrow_waypoint_array(route):
        route_node_arrow_waypoint_array = []
        for arrow_code in route.arrow_codes[1:]:
            waypoint_id1, _ = Arrow.split_arrow_code(arrow_code)
            route_node_arrow_waypoint_array.append({"waypoint_id": waypoint_id1, "arrow_code": arrow_code})
        return route_node_arrow_waypoint_array

    def get_route_waypoint_ids(self, route):
        print("Use route.get_waypoint_ids()")
        return self.get_waypoint_ids(route)

    def get_waypoint_ids(self, route):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.start_waypoint_id
        goal_waypoint_id = route.goal_waypoint_id
        waypoint_ids = []
        for i, arrow_code in enumerate(arrow_codes):
            arrow_waypoint_ids = self.__arrow.get_waypoint_ids(arrow_code)
            js = 0
            if i == 0 and start_waypoint_id in arrow_waypoint_ids:
                js = arrow_waypoint_ids.index(start_waypoint_id)
            je = len(arrow_waypoint_ids)
            if i == len(arrow_codes)-1 and goal_waypoint_id in arrow_waypoint_ids:
                je = arrow_waypoint_ids.index(goal_waypoint_id) + 1
            for j in range(js+1, je):
                waypoint_ids.append(arrow_waypoint_ids[j - 1])
            if arrow_code == arrow_codes[-1]:
                waypoint_ids.append(arrow_waypoint_ids[je - 1])
        return waypoint_ids

    def get_route_length(self, route):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.start_waypoint_id
        goal_waypoint_id = route.goal_waypoint_id
        length = 0.0
        for i, arrow_code in enumerate(arrow_codes):
            waypoint_ids = self.__arrow.get_waypoint_ids(arrow_code)
            js = 0
            if i == 0 and start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if i == len(arrow_codes)-1 and goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1

            if js == 0 and je == len(waypoint_ids):
                length += self.__arrow.get_length(arrow_code)
            else:
                for j in range(js+1, je):
                    length += self.__arrow.get_distance(
                        self.__waypoint.get_np_position(waypoint_ids[j - 1]),
                        self.__waypoint.get_np_position(waypoint_ids[j]))
        return length

    def __is_directly_reach(self, arrow_code, start_waypoint_id, goal_waypoint_id, reverse):
        waypoint_ids = self.__arrow.get_waypoint_ids(arrow_code)
        is_directly_reach = waypoint_ids.index(start_waypoint_id) <= waypoint_ids.index(goal_waypoint_id)
        return is_directly_reach if not reverse else not is_directly_reach

    def get_distance_of_waypoints(self, waypoint_ids):
        distance = 0.0
        for i in range(1, len(waypoint_ids)):
            distance += self.__arrow.get_distance(
                self.__waypoint.get_np_position(waypoint_ids[i]), self.__waypoint.get_np_position(waypoint_ids[i - 1]))
        return distance

    def get_sliced_route(self, route, length):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.start_waypoint_id
        goal_waypoint_id = route.goal_waypoint_id
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
                    self.__waypoint.get_np_position(
                        waypoint_ids[i]), self.__waypoint.get_np_position(waypoint_ids[i-1]))
                if length <= total_length:
                    return Route.new_route(start_waypoint_id, sliced_goal_waypoint_id, sliced_arrow_codes)
                if len(sliced_arrow_codes) == 0 or sliced_arrow_codes[-1] != arrow_code:
                    sliced_arrow_codes.append(arrow_code)
                sliced_goal_waypoint_id = waypoint_ids[i]
        return route

    def get_shortest_routes(self, start, goals, cost_limit=float_info.max, reverse=False):
        """
        # Dijkstra's algorithm
        """
        next_arrows = self.__arrow.get_to_arrows()
        waypoint_id_end = self.__arrow.get_waypoint_ids(start["arrow_code"])[-1]
        local_start_waypoint_id = start["waypoint_id"]
        local_goal_waypoint_id = waypoint_id_end
        if reverse:
            next_arrows = self.__arrow.get_from_arrows()
            waypoint_id_end = self.__arrow.get_waypoint_ids(start["arrow_code"])[0]
            local_start_waypoint_id = waypoint_id_end
            local_goal_waypoint_id = start["waypoint_id"]

        local_route = Route.new_route(local_start_waypoint_id, local_goal_waypoint_id, [start["arrow_code"]])
        cost_start_to_end = self.__getRouteCost(local_route)

        goal_arrow_candidates = self.__get_goal_arrow_candidates(goals, reverse)

        checked_arrow_code = []
        current_arrow_code = start["arrow_code"]

        end_arrows = {current_arrow_code: {"cost": cost_start_to_end, "prev_arrows": []}}
        shortest_routes = {}

        # check same arrow
        for goal_points in goal_arrow_candidates.values():
            for goal_candidate in goal_points.values():
                if start["arrow_code"] == goal_candidate["arrow_code"]:
                    if self.__is_directly_reach(
                            start["arrow_code"], start["waypoint_id"], goal_candidate["waypoint_id"], reverse):
                        local_start_waypoint_id = start["waypoint_id"]
                        local_goal_waypoint_id = goal_candidate["waypoint_id"]
                        if reverse:
                            local_start_waypoint_id = goal_candidate["waypoint_id"]
                            local_goal_waypoint_id = start["waypoint_id"]
                        cost_start_to_goal = self.__getRouteCost(Route.new_route(
                            local_start_waypoint_id, local_goal_waypoint_id, [start["arrow_code"]]))
                        shortest_routes[goal_candidate["goal_id"]] = Route.new_route(
                            start["waypoint_id"], goal_candidate["waypoint_id"], [start["arrow_code"]])
                        shortest_routes[goal_candidate["goal_id"]].update({
                            "goal_id": goal_candidate["goal_id"],
                            "cost": cost_start_to_goal
                        })

        while True:
            next_arrow_codes = next_arrows[current_arrow_code]

            if 0 == len(next_arrow_codes):
                end_arrows[current_arrow_code]["cost"] = float_info.max

                end_arrows_filtered = dict(filter(lambda x: x[1]["cost"] < cost_limit, end_arrows.items()))
                if len(end_arrows_filtered) == 0:
                    break
                current_arrow_code = min(end_arrows_filtered.items(), key=lambda x: x[1]["cost"])[0]

                continue

            for next_arrow_id in next_arrow_codes:
                if next_arrow_id in checked_arrow_code and next_arrow_id != start["arrow_code"]:
                    continue
                if next_arrow_id in end_arrows:
                    continue

                # update end_arrows
                end_arrows[next_arrow_id] = {
                    "cost": end_arrows[current_arrow_code]["cost"] + self.__getRouteCost(
                        self.__arrow_code_to_route(current_arrow_code)),
                    "prev_arrows": [current_arrow_code] + end_arrows[current_arrow_code]["prev_arrows"]
                }

                for goal_candidate in goal_arrow_candidates.get(next_arrow_id, {}).values():
                    if end_arrows[next_arrow_id]["cost"] + goal_candidate["cost"] < cost_limit:
                        shortest_route = Route.new_route(
                            start["waypoint_id"], goal_candidate["waypoint_id"],
                            [next_arrow_id] + end_arrows[next_arrow_id]["prev_arrows"])
                        shortest_route.update({
                            "goal_id": goal_candidate["goal_id"],
                            "cost": end_arrows[next_arrow_id]["cost"] + goal_candidate["cost"],
                        })
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
                end_waypoint_id = self.__arrow.get_waypoint_ids(arrow_code)[-1]
            else:
                end_waypoint_id = self.__arrow.get_waypoint_ids(arrow_code)[0]
            cost = self.__getRouteCost(Route.new_route(end_waypoint_id, goal["waypoint_id"], [arrow_code]))

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
        print("Use Route.encode_route().")
        return Route.encode_route(route)

    @staticmethod
    def split_route_code(route_code):
        print("Use Route.decode_route_code().")
        return Route.decode_route_code(route_code)

    @staticmethod
    def encode_route(route):
        joined_arrow_codes = Arrow.CONST.DELIMITER.join(list(map(
            lambda x: x.split(Arrow.CONST.DELIMITER)[0],
            route.arrow_codes)) + [route.arrow_codes[-1].split(Arrow.CONST.DELIMITER)[-1]])
        route_code = ROUTE.DELIMITER.join(map(
            str, [route.start_waypoint_id, joined_arrow_codes, route.goal_waypoint_id]))
        return route_code

    @staticmethod
    def decode_route_code(route_code):
        start_waypoint_id, joined_arrow_codes, goal_waypoint_id = route_code.split(ROUTE.DELIMITER)
        waypoint_ids = joined_arrow_codes.split(Arrow.CONST.DELIMITER)
        arrow_codes = []
        for i in range(1, len(waypoint_ids)):
            arrow_codes.append(Arrow.CONST.DELIMITER.join(waypoint_ids[i-1:i+1]))
        return Route.new_route(start_waypoint_id, goal_waypoint_id, arrow_codes)

    def get_speed_limits(self, route):
        waypoint_ids = self.get_route_waypoint_ids(route)
        speed_limits = list(map(self.__waypoint.get_speed_limit, waypoint_ids))
        return speed_limits

    def get_moved_position(self, position, distance, route):
        arrow_code, waypoint_id, _, moved_distance = \
            self.__arrow.get_point_to_arrows(position, route.arrow_codes)

        arrow_codes = route.arrow_codes[route.arrow_codes.index(arrow_code):]
        arrow_waypoint_array = self.get_arrow_waypoint_array(self.new_route(
            waypoint_id,
            route.goal_waypoint_id,
            arrow_codes
        ))

        for i in range(0, len(arrow_waypoint_array)-1):
            p1 = self.__waypoint.get_np_position(arrow_waypoint_array[i]["waypoint_id"])
            p2 = self.__waypoint.get_np_position(arrow_waypoint_array[i+1]["waypoint_id"])
            d = self.__arrow.get_distance(p1, p2)
            if distance <= moved_distance + d:
                vector_12 = np.subtract(p2, p1)
                return np.add(p1, d * vector_12 / np.linalg.norm(vector_12)),\
                    arrow_waypoint_array[i+1]["waypoint_id"],\
                    arrow_waypoint_array[i+1]["arrow_code"]
            moved_distance += d

        return self.__waypoint.get_np_position(arrow_waypoint_array[-1]["waypoint_id"]), \
            arrow_waypoint_array[-1]["waypoint_id"], \
            arrow_waypoint_array[-1]["arrow_code"]
