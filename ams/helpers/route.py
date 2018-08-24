#!/usr/bin/env python
# coding: utf-8

import json
from sys import float_info
from math import modf
from time import time

from ams import logger
from ams.helpers import Location, Position, Vector, Waypoint, Arrow, Orientation
from ams.structures import ROUTE, RouteDetail, AutowareMessage
from ams.structures import Route as Structure
from ams.structures import Routes as Structures


class Route(object):

    CONST = ROUTE

    @classmethod
    def new_route(cls, waypoint_ids, arrow_codes, delimiters=None):
        if delimiters is None:
            return Structure.new_data(
                waypoint_ids=waypoint_ids,
                arrow_codes=arrow_codes
            )
        return Structure.new_data(
            waypoint_ids=waypoint_ids,
            arrow_codes=arrow_codes,
            delimiters=delimiters
        )

    validate_route = Structure.validate_data
    get_errors = Structure.get_errors

    @classmethod
    def new_point_route(cls, waypoint_id, arrow_code):
        return Structure.new_data(
            waypoint_ids=[waypoint_id, waypoint_id],
            arrow_codes=[arrow_code]
        )

    @classmethod
    def new_point_route_from_location(cls, location):
        return Route.new_point_route(location.waypoint_id, location.arrow_code)

    @classmethod
    def new_routes(cls, routes):
        return Structures.new_data(routes)

    @classmethod
    def load(cls, path):
        with open(path, "r") as f:
            data = json.load(f)
            routes = dict(map(lambda x: (x["ID"], x), data["routes"]))
        return routes

    @classmethod
    def get_route_code(cls, route_id, routes):
        return routes[route_id]["code"]

    @classmethod
    def get_route(cls, route_id, routes):
        start_waypoint_id, arrow_codes, goal_waypoint_id = \
            cls.get_route_code(route_id, routes).split(ROUTE.DELIMITERS.WAYPOINT_ON_ARROW)
        return cls.new_route(
            waypoint_ids=[start_waypoint_id, goal_waypoint_id], arrow_codes=arrow_codes)

    @classmethod
    def get_route_ids(cls, routes):
        return routes.keys()

    @classmethod
    def get_route_from_arrow_code(cls, arrow_code, arrows):
        arrow = Arrow.get_arrow(arrow_code, arrows)
        return cls.new_route([arrow["waypointIDs"][0], arrow["waypointIDs"][-1]], [arrow_code])

    @classmethod
    def split_arrow_codes(cls, joined_arrow_code):
        arrow_codes = []
        for joined_foreward_arrow_code in joined_arrow_code.split(ROUTE.DELIMITERS.BACKWARD):
            waypoint_ids = joined_foreward_arrow_code.split(ROUTE.DELIMITERS.FOREWARD)
            for i in range(1, len(waypoint_ids)):
                arrow_codes.append(Arrow.CONST.DELIMITER.join(waypoint_ids[i-1:i+1]))
        return arrow_codes

    @classmethod
    def encode(cls, route):
        if "delimiters" not in route:
            delimiters = [":"] + [">"]*len(route.arrow_codes) + [":"]
        else:
            delimiters = route.delimiters

        route_parts = [route.waypoint_ids[0]]
        waypoint_id_index = 1
        arrow_code_index = 0
        for i, delimiter in enumerate(delimiters):
            if delimiter == ROUTE.DELIMITERS.WAYPOINT_ON_ARROW:
                if i == len(delimiters) - 1 or delimiters[i+1] == ROUTE.DELIMITERS.WAYPOINT_ON_ARROW:
                    route_parts.append(route.waypoint_ids[waypoint_id_index])
                    waypoint_id_index += 1
            elif delimiter == ROUTE.DELIMITERS.FOREWARD:
                end_waypoint_ids = Arrow.split_arrow_code(route.arrow_codes[arrow_code_index])
                end_waypoint_ids.insert(1, ROUTE.DELIMITERS.FOREWARD)
                if ROUTE.DELIMITERS.FOREWARD+end_waypoint_ids[0] in route_parts[-1]:
                    route_parts[-1] += "".join(end_waypoint_ids[1:])
                else:
                    route_parts.append("".join(end_waypoint_ids))
                arrow_code_index += 1
            elif delimiter == ROUTE.DELIMITERS.BACKWARD:
                end_waypoint_ids = Arrow.split_arrow_code(route.arrow_codes[arrow_code_index])
                end_waypoint_ids.reverse()
                end_waypoint_ids.insert(1, ROUTE.DELIMITERS.BACKWARD)
                if ROUTE.DELIMITERS.BACKWARD+end_waypoint_ids[0] in route_parts[-1]:
                    route_parts[-1] += "".join(end_waypoint_ids[1:])
                else:
                    route_parts.append("".join(end_waypoint_ids))
                arrow_code_index += 1
            else:
                raise ValueError("Unknown delimiter: {}".format(delimiter))

        return ROUTE.DELIMITERS.WAYPOINT_ON_ARROW.join(route_parts)

    @classmethod
    def decode(cls, route_code):
        route_parts = route_code.split(ROUTE.DELIMITERS.WAYPOINT_ON_ARROW)
        waypoint_ids = []
        arrow_codes = []
        delimiters = []
        for i, route_part in enumerate(route_parts):
            if any(map(lambda x: x in route_part, ROUTE.DELIMITERS)):
                if ROUTE.DELIMITERS.FOREWARD in route_part:
                    arrow_parts = route_part.split(ROUTE.DELIMITERS.FOREWARD)
                    for j in range(len(arrow_parts)-1):
                        end_waypoint_ids = arrow_parts[j:j+2]
                        arrow_codes.append(Arrow.CONST.DELIMITER.join(end_waypoint_ids))
                        delimiters.append(ROUTE.DELIMITERS.FOREWARD)
                elif ROUTE.DELIMITERS.BACKWARD in route_part:
                    arrow_parts = route_part.split(ROUTE.DELIMITERS.BACKWARD)
                    for j in range(len(arrow_parts)-1):
                        end_waypoint_ids = arrow_parts[j:j+2]
                        end_waypoint_ids.reverse()
                        arrow_codes.append(Arrow.CONST.DELIMITER.join(end_waypoint_ids))
                        delimiters.append(ROUTE.DELIMITERS.BACKWARD)
                else:
                    raise ValueError("Unknown route_part discription: {}".format(route_part))
            else:
                waypoint_ids.append(route_part)
            if i < len(route_parts) - 1:
                delimiters.append(ROUTE.DELIMITERS.WAYPOINT_ON_ARROW)

        return Route.new_route(waypoint_ids, arrow_codes, delimiters)

    @classmethod
    def get_pose_and_velocity_set(cls, route_code, arrows, waypoints):
        route = cls.decode(route_code)

        waypoint_ids_set = []
        action_delimitiers = list(filter(lambda x: x != ROUTE.DELIMITERS.WAYPOINT_ON_ARROW, route.delimiters))
        for i, arrow_code in enumerate(route.arrow_codes):
            waypoint_ids_set.append(Arrow.get_waypoint_ids(arrow_code, arrows))
            if action_delimitiers[i] == ROUTE.DELIMITERS.BACKWARD:
                waypoint_ids_set[-1].reverse()

        pose_and_velocity_set = []
        for i, waypoint_ids in enumerate(waypoint_ids_set):
            start_waypoint_id = route.waypoint_ids[i]
            goal_waypoint_id = route.waypoint_ids[i+1]
            start_index = waypoint_ids.index(start_waypoint_id)
            goal_index = len(waypoint_ids) - 1 - list(reversed(waypoint_ids)).index(goal_waypoint_id)

            for waypoint_id in waypoint_ids[start_index:goal_index+1]:
                velocity = Waypoint.get_velocity(waypoint_id, waypoints)
                pose = Arrow.get_pose(route.arrow_codes[i], waypoint_id, arrows, waypoints)
                if action_delimitiers[i] == ROUTE.DELIMITERS.BACKWARD:
                    velocity *= -1.0
                    pose.orientation = Orientation.get_inverted(pose.orientation)
                pose_and_velocity_set.append([pose, velocity])
        return pose_and_velocity_set

    @classmethod
    def get_lane_array(cls, route_code, arrows, waypoints, current_time=None):
        pose_and_velocity_set = Route.get_pose_and_velocity_set(route_code, arrows, waypoints)

        header = AutowareMessage.Header.get_template()
        if current_time is None:
            current_time = time()
        nsec, sec = modf(current_time)
        header.stamp.secs = int(sec)
        header.stamp.nsecs = int(nsec * (10 ** 9))
    
        lane_array = AutowareMessage.LaneArray.get_template()
        lane_array.lanes[0].header = header
        lane_array.lanes[0].waypoints = []
    
        for pose, velocity in pose_and_velocity_set:
            waypoint = AutowareMessage.LaneArray.Lane.Waypoint.get_template()
            waypoint.pose.header = header
            waypoint.pose.pose.position.x = pose.position.x
            waypoint.pose.pose.position.y = pose.position.y
            waypoint.pose.pose.position.z = pose.position.z
            waypoint.pose.pose.orientation.z = pose.orientation.quaternion.z
            waypoint.pose.pose.orientation.w = pose.orientation.quaternion.w
    
            waypoint.twist.header = header
            waypoint.twist.twist.linear.x = velocity

            lane_array.lanes[0].waypoints.append(waypoint)
    
        return lane_array

    @classmethod
    def get_length(cls, route, arrows, waypoints):
        arrow_codes = route.arrow_codes
        start_waypoint_id = route.waypoint_ids[0]
        goal_waypoint_id = route.waypoint_ids[-1]
        length = 0.0
        for i, arrow_code in enumerate(arrow_codes):
            waypoint_ids = Arrow.get_waypoint_ids(arrow_code, arrows)
            js = 0
            if i == 0 and start_waypoint_id in waypoint_ids:
                js = waypoint_ids.index(start_waypoint_id)
            je = len(waypoint_ids)
            if i == len(arrow_codes)-1 and goal_waypoint_id in waypoint_ids:
                je = waypoint_ids.index(goal_waypoint_id) + 1

            if js == 0 and je == len(waypoint_ids):
                length += Arrow.get_length(arrow_code, arrows)
            else:
                for j in range(js+1, je):
                    length += Arrow.get_distance(
                        Waypoint.get_position(waypoint_ids[j - 1], waypoints),
                        Waypoint.get_position(waypoint_ids[j], waypoints))
        return length

    @classmethod
    def __get_goal_arrow_candidates(cls, goals, reverse, arrows, waypoints, cost_function):
        goal_arrow_candidates = {}
        for goal in goals:
            goal_id = goal["goal_id"]
            arrow_code = goal["arrow_code"]
            if reverse:
                end_waypoint_id = Arrow.get_waypoint_ids(arrow_code, arrows)[-1]
            else:
                end_waypoint_id = Arrow.get_waypoint_ids(arrow_code, arrows)[0]
            cost = cost_function(
                cls.new_route([end_waypoint_id, goal["waypoint_id"]], [arrow_code]), arrows, waypoints)

            goal_points = goal_arrow_candidates.get(arrow_code, {})
            goal_points[goal_id] = {
                "arrow_code": arrow_code,
                "goal_id": goal_id,
                "waypoint_id": goal["waypoint_id"],
                "cost": cost,
            }
            goal_arrow_candidates[arrow_code] = goal_points

        return goal_arrow_candidates

    @classmethod
    def __is_directly_reach(cls, arrow_code, start_waypoint_id, goal_waypoint_id, reverse, arrows):
        waypoint_ids = Arrow.get_waypoint_ids(arrow_code, arrows)
        is_directly_reach = waypoint_ids.index(start_waypoint_id) <= waypoint_ids.index(goal_waypoint_id)
        return is_directly_reach if not reverse else not is_directly_reach

    @classmethod
    def get_shortest_routes(
            cls, start, goals, arrows, to_arrows, from_arrows, waypoints, cost_function,
            cost_limit=ROUTE.COST_LIMIT, reverse=False):
        """
        # Dijkstra's algorithm
        """
        next_arrows = to_arrows
        local_start_waypoint_id = start["waypoint_id"]
        local_goal_waypoint_id = Arrow.get_waypoint_ids(start["arrow_code"], arrows)[-1]
        if reverse:
            next_arrows = from_arrows
            local_start_waypoint_id = Arrow.get_waypoint_ids(start["arrow_code"], arrows)[0]
            local_goal_waypoint_id = start["waypoint_id"]

        local_route = cls.new_route([local_start_waypoint_id, local_goal_waypoint_id], [start["arrow_code"]])
        cost_start_to_end = cost_function(local_route, arrows, waypoints)

        goal_arrow_candidates = cls.__get_goal_arrow_candidates(goals, reverse, arrows, waypoints, cost_function)

        checked_arrow_code = []
        current_arrow_code = start["arrow_code"]

        end_arrows = {current_arrow_code: {"cost": cost_start_to_end, "prev_arrows": []}}
        shortest_routes = {}

        for goal_points in goal_arrow_candidates.values():
            for goal_candidate in goal_points.values():
                if start["arrow_code"] == goal_candidate["arrow_code"]:
                    if cls.__is_directly_reach(
                            start["arrow_code"], start["waypoint_id"], goal_candidate["waypoint_id"], reverse, arrows):
                        local_start_waypoint_id = start["waypoint_id"]
                        local_goal_waypoint_id = goal_candidate["waypoint_id"]
                        if reverse:
                            local_start_waypoint_id = goal_candidate["waypoint_id"]
                            local_goal_waypoint_id = start["waypoint_id"]
                        cost_start_to_goal = cost_function(
                            cls.new_route([local_start_waypoint_id, local_goal_waypoint_id], [start["arrow_code"]]),
                            arrows, waypoints)
                        shortest_routes[goal_candidate["goal_id"]] = cls.new_route(
                            [start["waypoint_id"], goal_candidate["waypoint_id"]], [start["arrow_code"]])
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

                end_arrows[next_arrow_id] = {
                    "cost": end_arrows[current_arrow_code]["cost"] + cost_function(
                        Route.get_route_from_arrow_code(current_arrow_code, arrows), arrows, waypoints),
                    "prev_arrows": [current_arrow_code] + end_arrows[current_arrow_code]["prev_arrows"]
                }

                for goal_candidate in goal_arrow_candidates.get(next_arrow_id, {}).values():
                    if end_arrows[next_arrow_id]["cost"] + goal_candidate["cost"] < cost_limit:
                        shortest_route = cls.new_route(
                            [start["waypoint_id"], goal_candidate["waypoint_id"]],
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

        if reverse:
            for route_id in shortest_routes:
                shortest_routes[route_id]["start_waypoint_id"] = shortest_routes[route_id]["goal_waypoint_id"]
                shortest_routes[route_id]["goal_waypoint_id"] = start["waypoint_id"]
        else:
            for route_id in shortest_routes:
                shortest_routes[route_id]["arrow_codes"].reverse()

        return shortest_routes
