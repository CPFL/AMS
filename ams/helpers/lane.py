#!/usr/bin/env python
# coding: utf-8

import json
import math
from copy import copy, deepcopy

from ams.helpers import Position, Vector, Rpy, Location, Waypoint
from ams.structures import LANE, Orientation, Pose


class Lane(object):

    CONST = LANE

    @classmethod
    def load(cls, path):
        with open(path, "r") as f:
            data = json.load(f)
        return data["lanes"], data["toLanes"], data["fromLanes"]

    @classmethod
    def get_lane_codes(cls, lanes):
        return tuple(lanes.keys())

    @classmethod
    def get_lane(cls, lane_code, lanes):
        return deepcopy(lanes[lane_code])

    @classmethod
    def get_waypoint_ids(cls, lane_code, lanes):
        return copy(lanes[lane_code]["waypointIDs"])

    @classmethod
    def get_length(cls, lane_code, lanes):
        return lanes[lane_code]["length"]

    @classmethod
    def get_yaw(cls, lane_code, waypoint_id, lanes, waypoints):
        waypoint_ids = Lane.get_waypoint_ids(lane_code, lanes)
        index = waypoint_ids.index(waypoint_id)
        sub_vector = Vector.get_sub_vector(*map(
            Position.get_vector,
            (
                Waypoint.get_position(waypoint_ids[min([len(waypoint_ids) - 1, index + 1])], waypoints),
                Waypoint.get_position(waypoint_ids[max([0, index - 1])], waypoints)
            )
        ))
        atan2 = math.atan2(sub_vector[1], sub_vector[0])
        return atan2 if 0 < atan2 else 2.0 * math.pi + atan2

    @classmethod
    def get_orientation(cls, lane_code, waypoint_id, lanes, waypoints):
        return Orientation.new_data(
            quaternion=dict(zip(
                ["w", "x", "y", "z"],
                Rpy.to_quaternion([0, 0, 1], Lane.get_yaw(lane_code, waypoint_id, lanes, waypoints)))),
            rpy=Rpy.Structure.new_data(
                roll=None,
                pitch=None,
                yaw=Lane.get_yaw(lane_code, waypoint_id, lanes, waypoints)
            )
        )

    @classmethod
    def get_pose(cls, lane_code, waypoint_id, lanes, waypoints):
        return Pose.new_data(
            position=Waypoint.get_position(waypoint_id, waypoints),
            orientation=Lane.get_orientation(lane_code, waypoint_id, lanes, waypoints)
        )

    @classmethod
    def get_heading(cls, lane_code, waypoint_id, lanes, waypoints):
        return math.degrees(cls.get_yaw(lane_code, waypoint_id, lanes, waypoints))

    @classmethod
    def get_distance(cls, position1, position2):
        return Vector.get_norm(Vector.get_sub_vector(*map(Position.get_vector, (position1, position2))))

    @classmethod
    def get_point_to_edge(cls, position, edge_position1, edge_position2):
        vector_12 = Vector.get_sub_vector(*map(Position.get_vector, (edge_position2, edge_position1)))
        vector_1p = Vector.get_sub_vector(*map(Position.get_vector, (position, edge_position1)))

        # get unit vector
        len_12 = Vector.get_norm(vector_12)
        unit_vector_12 = Vector.get_div_vector(vector_12, [len_12]*len(vector_12))

        # dot product
        distance1x = Vector.get_dot(unit_vector_12, vector_1p)
        if len_12 < distance1x:
            return edge_position2, Helper.get_distance(position, edge_position2)
        elif distance1x < 0.0:
            return edge_position1, Helper.get_distance(position, edge_position1)
        else:
            distance1x_vector_12 = Vector.get_mul_vector(unit_vector_12, [distance1x]*len(unit_vector_12))
            matched_position = Position.new_position(
                *Vector.get_add_vector(Position.get_vector(edge_position1), distance1x_vector_12))
            return matched_position, Helper.get_distance(position, matched_position)

    @classmethod
    def get_point_to_waypoints(cls, position, waypoint_ids, waypoints):
        matched_waypoints = {}
        prev_position = None
        for waypoint_id in waypoint_ids:
            next_position = Waypoint.get_position(waypoint_id, waypoints)
            if prev_position is not None:
                position_on_edge, distance = Helper.get_point_to_edge(position, prev_position, next_position)
                matched_waypoints[waypoint_id] = {
                    "waypoint_id": waypoint_id,
                    "position": position_on_edge,
                    "distance": distance
                }
            prev_position = next_position

        most_matched_waypoint = min(matched_waypoints.values(), key=lambda x: x["distance"])
        return most_matched_waypoint["waypoint_id"], \
            most_matched_waypoint["position"], \
            most_matched_waypoint["distance"]

    @classmethod
    def get_point_to_lane(cls, position, lane_code, lanes, waypoints):
        return cls.get_point_to_waypoints(position, cls.get_waypoint_ids(lane_code, lanes), waypoints)

    @classmethod
    def get_point_to_lanes(cls, position, lanes, waypoints, lane_codes=None):
        matched_waypoints = {}
        if lane_codes is None:
            lane_codes = cls.get_lane_codes(lanes)
        for lane_code in lane_codes:
            waypoint_id, position, distance = cls.get_point_to_lane(position, lane_code, lanes, waypoints)
            matched_waypoints[waypoint_id] = {
                "lane_code": lane_code,
                "waypoint_id": waypoint_id,
                "position": position,
                "distance": distance
            }

        most_matched_waypoint = min(matched_waypoints.values(), key=lambda x: x["distance"])
        return most_matched_waypoint["lane_code"],\
            most_matched_waypoint["waypoint_id"],\
            most_matched_waypoint["position"], \
            most_matched_waypoint["distance"]

    @classmethod
    def filter_by_lane_code(cls, lanes, lane_codes):
        filtered_lanes = {}
        filtered_to_lanes = {}
        filtered_from_lanes = {}
        for lane_code in lane_codes:
            filtered_lanes[lane_code] = cls.get_lane(lane_code, lanes)

        for i in range(1, len(lane_codes)):
            filtered_to_lanes[lane_codes[i - 1]] = [lane_codes[i]]
            filtered_from_lanes[lane_codes[i]] = [lane_codes[i - 1]]

        return filtered_lanes, filtered_to_lanes, filtered_from_lanes

    @classmethod
    def get_lane_codes_by_waypoint_id(cls, waypoint_id, lanes):
        return list(map(lambda x: x[0], filter(lambda x: waypoint_id in x[1]["waypointIDs"], lanes.items())))

    @classmethod
    def get_lane_codes_set_by_waypoint_ids(cls, waypoint_ids, lanes):
        lane_codes_set = [[]]
        for waypoint_id in waypoint_ids:
            lane_codes = cls.get_lane_codes_by_waypoint_id(waypoint_id, lanes)
            for lane_code in lane_codes:
                if cls.get_waypoint_ids(lane_code, lanes)[-1] in waypoint_ids:
                    for i in range(len(lane_codes_set)):
                        lane_codes_set[i].append(lane_code)
        return lane_codes_set

    @classmethod
    def get_locations(cls, waypoint_id, lanes):
        lane_codes = cls.get_lane_codes_by_waypoint_id(waypoint_id, lanes)
        return list(map(lambda x: Location.new_location(waypoint_id, x), lane_codes))

    @classmethod
    def split_lane_code(cls, lane_code):
        return lane_code.split(LANE.DELIMITER)
