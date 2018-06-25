#!/usr/bin/env python
# coding: utf-8

import json
import math

from ams.helpers import Position, Vector, Rpy, Location
from ams.structures import ARROW, Orientation


class Arrow(object):

    CONST = ARROW

    def __init__(self, waypoint=None):
        self.waypoint = waypoint
        self.__arrows = None
        self.__to_arrows = None
        self.__from_arrows = None

    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.set_arrows(data["arrows"], data["toArrows"], data["fromArrows"])
        return True

    def set_waypoint(self, waypoint):
        self.waypoint = waypoint

    def connect_to_redis(self, _host, _port, _dbname):
        return self

    def set_arrows(self, arrows, to_arrows, from_arrows):
        self.__arrows = arrows
        self.__to_arrows = to_arrows
        self.__from_arrows = from_arrows

    def get_arrow_codes(self):
        return list(self.__arrows.keys())

    def get_arrow(self, arrow_code):
        return self.__arrows[arrow_code]

    def get_arrows(self):
        return self.__arrows

    def get_to_arrows(self):
        return self.__to_arrows

    def get_from_arrows(self):
        return self.__from_arrows

    def get_waypoint_ids(self, arrow_code):
        return self.__arrows[arrow_code]["waypointIDs"]

    def get_length(self, arrow_code):
        return self.__arrows[arrow_code]["length"]

    @staticmethod
    def get_distance(position1, position2):
        return Vector.get_norm(Vector.get_sub_vector(*map(Position.get_vector, (position1, position2))))

    def get_yaw(self, arrow_code, waypoint_id):
        waypoint_ids = self.__arrows[arrow_code]["waypointIDs"]
        index = waypoint_ids.index(waypoint_id)
        sub_vector = Vector.get_sub_vector(*map(
            Position.get_vector,
            (
                self.waypoint.get_position(waypoint_ids[max([0, index - 1])]),
                self.waypoint.get_position(waypoint_ids[min([len(waypoint_ids) - 1, index + 1])])
            )
        ))
        return math.pi+math.atan2(sub_vector[0], sub_vector[1])

    def get_orientation(self, arrow_code, waypoint_id):
        return Orientation.new_data(
            quaternion=dict(zip(
                ["w", "x", "y", "z"],
                Rpy.to_quaternion([0, 0, 1], self.get_yaw(arrow_code, waypoint_id)))),
            rpy=Rpy.Structure.new_data(
                roll=None,
                pitch=None,
                yaw=self.get_yaw(arrow_code, waypoint_id)
            )
        )

    def get_heading(self, arrow_code, waypoint_id):
        return math.degrees(self.get_yaw(arrow_code, waypoint_id))

    def get_point_to_edge(self, position, edge_position1, edge_position2):
        vector_12 = Vector.get_sub_vector(*map(Position.get_vector, (edge_position2, edge_position1)))
        vector_1p = Vector.get_sub_vector(*map(Position.get_vector, (position, edge_position1)))

        # get unit vector
        len_12 = Vector.get_norm(vector_12)
        unit_vector_12 = Vector.get_div_vector(vector_12, [len_12]*len(vector_12))

        # dot product
        distance1x = Vector.get_dot(unit_vector_12, vector_1p)
        if len_12 < distance1x:
            return edge_position2, self.get_distance(position, edge_position2)
        elif distance1x < 0.0:
            return edge_position1, self.get_distance(position, edge_position1)
        else:
            distance1x_vector_12 = Vector.get_mul_vector(unit_vector_12, [distance1x]*len(unit_vector_12))
            matched_position = Position.new_position(
                *Vector.get_add_vector(Position.get_vector(edge_position1), distance1x_vector_12))
            return matched_position, self.get_distance(position, matched_position)

    def get_point_to_waypoints(self, position, waypoint_ids):
        matched_waypoints = {}
        prev_position = None
        for waypoint_id in waypoint_ids:
            next_position = self.waypoint.get_position(waypoint_id)
            if prev_position is not None:
                position_on_edge, distance = self.get_point_to_edge(position, prev_position, next_position)
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

    def get_point_to_arrow(self, position, arrow_code):
        return self.get_point_to_waypoints(position, self.get_waypoint_ids(arrow_code))

    def get_point_to_arrows(self, position, arrow_codes=None):
        matched_waypoints = {}
        if arrow_codes is None:
            arrow_codes = self.__arrows.keys()
        for arrow_code in arrow_codes:
            waypoint_id, position, distance = self.get_point_to_arrow(position, arrow_code)
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

    def get_arrow_codes_to_arrows(self, arrow_codes):
        arrows = {}
        to_arrows = {}
        from_arrows = {}
        for arrow_code in arrow_codes:
            arrows[arrow_code] = self.__arrows[arrow_code]

        for i in range(1, len(arrow_codes)):
            to_arrows[arrow_codes[i - 1]] = [arrow_codes[i]]
            from_arrows[arrow_codes[i]] = [arrow_codes[i - 1]]

        return arrows, to_arrows, from_arrows

    def get_locations_by_waypoint_id(self, waypoint_id):
        arrow_codes = self.get_arrow_codes_from_waypoint_id(waypoint_id)
        return list(map(lambda x: Location.new_location(waypoint_id, x), arrow_codes))

    def get_arrow_codes_from_waypoint_id(self, waypoint_id):
        return list(map(lambda x: x[0], filter(lambda x: waypoint_id in x[1]["waypointIDs"], self.__arrows.items())))

    def get_arrow_codes_set_from_waypoint_ids(self, waypoint_ids):
        arrow_codes_set = [[]]
        for waypoint_id in waypoint_ids:
            arrow_codes = self.get_arrow_codes_from_waypoint_id(waypoint_id)
            for arrow_code in arrow_codes:
                if self.__arrows[arrow_code]["waypointIDs"][-1] in waypoint_ids:
                    for i in range(len(arrow_codes_set)):
                        arrow_codes_set[i].append(arrow_code)
        return arrow_codes_set

    @staticmethod
    def split_arrow_code(arrow_code):
        return arrow_code.split(ARROW.DELIMITER)
