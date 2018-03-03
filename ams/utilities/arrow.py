#!/usr/bin/env python
# coding: utf-8

import json
import numpy as np
from ams.structures import ARROW


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
        return np.linalg.norm(np.subtract(position1, position2))

    def get_yaw(self, arrow_code, waypoint_id):
        waypoint_ids = self.__arrows[arrow_code]["waypointIDs"]
        index = waypoint_ids.index(waypoint_id)
        sub_position = np.subtract(
            self.waypoint.get_np_position(waypoint_ids[max([0, index - 1])]),
            self.waypoint.get_np_position(waypoint_ids[min([len(waypoint_ids) - 1, index + 1])]))
        return np.pi+np.arctan2(sub_position[0], sub_position[1])

    def get_heading(self, arrow_code, waypoint_id):
        return np.degrees(self.get_yaw(arrow_code, waypoint_id))

    def get_point_to_edge(self, np_position, edge_position1, edge_position2):
        vector_12 = np.subtract(edge_position2, edge_position1)
        vector_1p = np.subtract(np_position, edge_position1)

        # get unit vector
        len_12 = np.linalg.norm(vector_12)
        unit_vector_12 = vector_12 / len_12

        # dot product
        distance1x = np.dot(unit_vector_12, vector_1p)
        if len_12 < distance1x:
            return edge_position2, self.get_distance(np_position, edge_position2)
        elif distance1x < 0.0:
            return edge_position1, self.get_distance(np_position, edge_position1)
        else:
            distance1x_vector_12 = unit_vector_12 * distance1x
            position = np.add(edge_position1, distance1x_vector_12)
            return position, self.get_distance(np_position, position)

    def get_point_to_waypoints(self, np_position, waypoint_ids):
        matched_waypoints = {}
        prev_position = None
        for waypoint_id in waypoint_ids:
            position = self.waypoint.get_np_position(waypoint_id)
            if prev_position is not None:
                position_on_edge, distance = self.get_point_to_edge(np_position, prev_position, position)
                matched_waypoints[waypoint_id] = {
                    "waypoint_id": waypoint_id,
                    "position": position_on_edge,
                    "distance": distance
                }
            prev_position = position

        if len(matched_waypoints) == 0:
            raise Exception

        most_matched_waypoint = min(matched_waypoints.values(), key=lambda x: x["distance"])
        return most_matched_waypoint["waypoint_id"], \
            most_matched_waypoint["position"], \
            most_matched_waypoint["distance"]

    def get_point_to_arrow(self, np_position, arrow_code):
        return self.get_point_to_waypoints(np_position, self.get_waypoint_ids(arrow_code))

    def get_point_to_arrows(self, np_position, arrow_codes=None):
        matched_waypoints = {}
        if arrow_codes is None:
            arrow_codes = self.__arrows.keys()
        for arrow_code in arrow_codes:
            waypoint_id, position, distance = self.get_point_to_arrow(np_position, arrow_code)
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
