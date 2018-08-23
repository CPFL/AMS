#!/usr/bin/env python
# coding: utf-8

import json
import math

from ams.helpers import Position, Vector, Rpy, Location, Waypoint
from ams.structures import ARROW, Orientation, Pose


class Arrow(object):

    CONST = ARROW

    @classmethod
    def load(cls, path):
        with open(path, "r") as f:
            data = json.load(f)
        return data["arrows"], data["toArrows"], data["fromArrows"]

    @classmethod
    def get_arrow_codes(cls, arrows):
        return list(arrows.keys())

    @classmethod
    def get_arrow(cls, arrow_code, arrows):
        return arrows[arrow_code]

    @classmethod
    def get_waypoint_ids(cls, arrow_code, arrows):
        return arrows[arrow_code]["waypointIDs"]

    @classmethod
    def get_length(cls, arrow_code, arrows):
        return arrows[arrow_code]["length"]

    @classmethod
    def get_yaw(cls, arrow_code, waypoint_id, arrows, waypoints):
        waypoint_ids = Arrow.get_waypoint_ids(arrow_code, arrows)
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
    def get_orientation(cls, arrow_code, waypoint_id, arrows, waypoints):
        return Orientation.new_data(
            quaternion=dict(zip(
                ["w", "x", "y", "z"],
                Rpy.to_quaternion([0, 0, 1], Arrow.get_yaw(arrow_code, waypoint_id, arrows, waypoints)))),
            rpy=Rpy.Structure.new_data(
                roll=None,
                pitch=None,
                yaw=Arrow.get_yaw(arrow_code, waypoint_id, arrows, waypoints)
            )
        )

    @classmethod
    def get_pose(cls, arrow_code, waypoint_id, arrows, waypoints):
        return Pose.new_data(
            position=Waypoint.get_position(waypoint_id, waypoints),
            orientation=Arrow.get_orientation(arrow_code, waypoint_id, arrows, waypoints)
        )

    @classmethod
    def get_heading(cls, arrow_code, waypoint_id, arrows, waypoints):
        return math.degrees(cls.get_yaw(arrow_code, waypoint_id, arrows, waypoints))

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

        if len(matched_waypoints) == 0:
            raise Exception

        most_matched_waypoint = min(matched_waypoints.values(), key=lambda x: x["distance"])
        return most_matched_waypoint["waypoint_id"], \
            most_matched_waypoint["position"], \
            most_matched_waypoint["distance"]

    @classmethod
    def get_point_to_arrow(cls, position, arrow_code, arrows, waypoints):
        return cls.get_point_to_waypoints(position, cls.get_waypoint_ids(arrow_code, arrows), waypoints)

    @classmethod
    def get_point_to_arrows(cls, position, arrows, waypoints, arrow_codes=None):
        matched_waypoints = {}
        if arrow_codes is None:
            arrow_codes = cls.get_arrow_codes(arrows)
        for arrow_code in arrow_codes:
            waypoint_id, position, distance = cls.get_point_to_arrow(position, arrow_code, arrows, waypoints)
            matched_waypoints[waypoint_id] = {
                "arrow_code": arrow_code,
                "waypoint_id": waypoint_id,
                "position": position,
                "distance": distance
            }

        if len(matched_waypoints) == 0:
            raise Exception

        most_matched_waypoint = min(matched_waypoints.values(), key=lambda x: x["distance"])
        return most_matched_waypoint["arrow_code"],\
            most_matched_waypoint["waypoint_id"],\
            most_matched_waypoint["position"], \
            most_matched_waypoint["distance"]

    @classmethod
    def filter_by_arrow_code(cls, arrows, arrow_codes):
        filtered_arrows = {}
        filtered_to_arrows = {}
        filtered_from_arrows = {}
        for arrow_code in arrow_codes:
            filtered_arrows[arrow_code] = cls.get_arrow(arrow_code, arrows)

        for i in range(1, len(arrow_codes)):
            filtered_to_arrows[arrow_codes[i - 1]] = [arrow_codes[i]]
            filtered_from_arrows[arrow_codes[i]] = [arrow_codes[i - 1]]

        return filtered_arrows, filtered_to_arrows, filtered_from_arrows

    @classmethod
    def get_arrow_codes_by_waypoint_id(cls, waypoint_id, arrows):
        return list(map(lambda x: x[0], filter(lambda x: waypoint_id in x[1]["waypointIDs"], arrows.items())))

    @classmethod
    def get_arrow_codes_set_by_waypoint_ids(cls, waypoint_ids, arrows):
        arrow_codes_set = [[]]
        for waypoint_id in waypoint_ids:
            arrow_codes = cls.get_arrow_codes_by_waypoint_id(waypoint_id, arrows)
            for arrow_code in arrow_codes:
                if cls.get_waypoint_ids(arrow_code, arrows)[-1] in waypoint_ids:
                    for i in range(len(arrow_codes_set)):
                        arrow_codes_set[i].append(arrow_code)
        return arrow_codes_set

    @classmethod
    def get_locations(cls, waypoint_id, arrows):
        arrow_codes = cls.get_arrow_codes_by_waypoint_id(waypoint_id, arrows)
        return list(map(lambda x: Location.new_location(waypoint_id, x), arrow_codes))

    @classmethod
    def split_arrow_code(cls, arrow_code):
        return arrow_code.split(ARROW.DELIMITER)
