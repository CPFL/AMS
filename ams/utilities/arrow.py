#!/usr/bin/env python
# coding: utf-8
import math
import json


class Arrow(object):
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

    def get_arrows(self):
        return self.__arrows

    def get_to_arrows(self):
        return self.__to_arrows

    def get_from_arrows(self):
        return self.__from_arrows

    def get_waypoint_ids(self, arrow_id):
        return self.__arrows[arrow_id]["waypointIDs"]

    @staticmethod
    def get_approximate_distance(lat1, lng1, lat2, lng2):
        distance = math.hypot(lat1-lat2, lng1-lng2)
        return distance

    def get_heading(self, arrow_id, waypoint_id):
        waypoint_ids = self.__arrows[arrow_id]["waypointIDs"]
        index = waypoint_ids.index(waypoint_id)
        lat1, lng1 = self.waypoint.get_latlng(waypoint_ids[max([0, index - 1])])
        lat2, lng2 = self.waypoint.get_latlng(waypoint_ids[min([len(waypoint_ids) - 1, index + 1])])
        return 180+math.degrees(math.atan2(lng1 - lng2, lat1 - lat2))

    def get_arrow_heading(self, arrow_id):
        """
        arrowの方角(deg)取得。
        :param arrow_id:
        :return:
        """
        return 180+math.degrees(math.atan2(
            self.__arrows[arrow_id]["lng1"] - self.__arrows[arrow_id]["lng2"],
            self.__arrows[arrow_id]["lat1"] - self.__arrows[arrow_id]["lat2"]
        ))

    def get_point_to_edge(self, point_lat, point_lng, edge_lat1, edge_lng1, edge_lat2, edge_lng2):
        lat12 = edge_lat2 - edge_lat1
        lng12 = edge_lng2 - edge_lng1
        lat1p = point_lat - edge_lat1
        lng1p = point_lng - edge_lng1

        # get unit vector
        len12 = math.hypot(lat12, lng12)
        u_lat12 = lat12 / len12
        u_lng12 = lng12 / len12

        # dot product
        distance1x = u_lat12 * lat1p + u_lng12 * lng1p
        if len12 < distance1x:
            return edge_lat2, edge_lng2, self.get_approximate_distance(point_lat, point_lng, edge_lat2, edge_lng2)
        elif distance1x < 0.0:
            return edge_lat1, edge_lng1, self.get_approximate_distance(point_lat, point_lng, edge_lat1, edge_lng1)
        else:
            lat = edge_lat1 + (u_lat12 * distance1x)
            lng = edge_lng1 + (u_lng12 * distance1x)
            return lat, lng, self.get_approximate_distance(point_lat, point_lng, lat, lng)

    def get_point_to_arrow(self, point_lat, point_lng, arrow_id):
        matched_waypoints = {}
        prev_lat, prev_lng = None, None
        for waypoint_id in self.__arrows[arrow_id]["waypointIDs"]:
            lat, lng = self.waypoint.get_latlng(waypoint_id)
            if None not in [prev_lat, prev_lng]:
                lat_on_edge, lng_on_edge, distance = self.get_point_to_edge(
                    point_lat, point_lng, prev_lat, prev_lng, lat, lng)
                matched_waypoints[waypoint_id] = {
                    "waypoint_id": waypoint_id,
                    "lat": lat_on_edge,
                    "lng": lng_on_edge,
                    "distance": distance
                }
            prev_lat, prev_lng = lat, lng

        if len(matched_waypoints) == 0:
            raise Exception

        most_matched_waypoint = min(matched_waypoints.values(), key=lambda x: x["distance"])
        return most_matched_waypoint["waypoint_id"], most_matched_waypoint["lat"], most_matched_waypoint["lng"], \
            most_matched_waypoint["distance"]

    def get_point_to_arrows(self, point_lat, point_lng, arrows=None):
        matched_waypoints = {}
        if arrows is None:
            arrows = self.__arrows
        for arrow_id in arrows:
            waypoint_id, lat, lng, distance = self.get_point_to_arrow(point_lat, point_lng, arrow_id)
            matched_waypoints[waypoint_id] = {
                "arrow_id": arrow_id,
                "waypoint_id": waypoint_id,
                "lat": lat,
                "lng": lng,
                "distance": distance
            }

        if len(matched_waypoints) == 0:
            raise Exception

        most_matched_waypoint = min(matched_waypoints.values(), key=lambda x: x["distance"])
        return most_matched_waypoint["arrow_id"], most_matched_waypoint["waypoint_id"], most_matched_waypoint["lat"], \
            most_matched_waypoint["lng"], most_matched_waypoint["distance"]

    def get_advanced_latlng_in_arrow(self, lat, lng, delta_distance, arrow_id, next_waypoint_id=None):
        if next_waypoint_id is None:
            next_waypoint_id, lat_on_edge, lng_on_edge, _ = self.get_point_to_arrow(lat, lng, arrow_id)
        else:
            lat_on_edge, lng_on_edge = lat, lng

        next_lat, next_lng = self.waypoint.get_latlng(next_waypoint_id)
        remaining_distance = self.get_approximate_distance(lat_on_edge, lng_on_edge, next_lat, next_lng)
        if delta_distance <= remaining_distance:
            lat12 = next_lat - lat_on_edge
            lng12 = next_lng - lng_on_edge
            len12 = math.hypot(lat12, lng12)
            u_lat12 = lat12 / len12
            u_lng12 = lng12 / len12
            advanced_lat = lat_on_edge + (u_lat12 * delta_distance)
            advanced_lng = lng_on_edge + (u_lng12 * delta_distance)
            return advanced_lat, advanced_lng, 0.0
        else:
            waypoint_ids = self.__arrows[arrow_id]["waypointIDs"]
            index = waypoint_ids.index(next_waypoint_id) + 1
            if index == len(waypoint_ids):
                return next_lat, next_lng, delta_distance - remaining_distance
            return self.get_advanced_latlng_in_arrow(
                next_lat, next_lng, delta_distance - remaining_distance, arrow_id, waypoint_ids[index])

    def get_advanced_latlng_in_arrows(self, lat, lng, delta_distance, arrows, next_arrows):
        arrow_id, _, _, _, _ = self.get_point_to_arrows(lat, lng, arrows)
        remaining_distance = delta_distance
        advanced_lat, advanced_lng = lat, lng
        while True:
            advanced_lat, advanced_lng, remaining_distance = self.get_advanced_latlng_in_arrow(
                advanced_lat, advanced_lng, remaining_distance, arrow_id)
            if remaining_distance == 0 or arrow_id not in next_arrows:
                return advanced_lat, advanced_lng, arrow_id
            arrow_id = next_arrows[arrow_id][0]

    def get_next_latlng(self, lat, lng, delta_distance, arrow_id=None, arrows=None, to_arrows=None):
        if arrows is None:
            arrows = self.__arrows
        if to_arrows is None:
            to_arrows = self.__to_arrows
        lat_on_arrow, lng_on_arrow = lat, lng
        if arrow_id is None:
            arrow_id, _, lat_on_arrow, lng_on_arrow, _ = self.get_point_to_arrows(lat, lng, arrows=arrows)
        remaining_distance = self.get_approximate_distance(
            lat_on_arrow, lng_on_arrow, arrows[arrow_id]["lat2"], arrows[arrow_id]["lng2"])
        if delta_distance <= remaining_distance:
            lat12 = arrows[arrow_id]["lat2"] - lat_on_arrow
            lng12 = arrows[arrow_id]["lng2"] - lng_on_arrow
            len12 = math.hypot(lat12, lng12)
            u_lat12 = lat12 / len12
            u_lng12 = lng12 / len12
            advanced_lat = lat_on_arrow + (u_lat12 * delta_distance)
            advanced_lng = lng_on_arrow + (u_lng12 * delta_distance)
            return advanced_lat, advanced_lng
        else:
            next_arrow = arrows[to_arrows[arrow_id][0]]
            return self.get_next_latlng(
                next_arrow["lat1"], next_arrow["lng1"], delta_distance - remaining_distance, next_arrow["name"],
                arrows=arrows, to_arrows=to_arrows)

    def get_advanced_latlng(self, lat, lng, delta_distance, arrow_id=None, arrows=None, to_arrows=None):
        print("get_advanced_latlng", arrows.keys(), to_arrows.keys())
        if arrows is None:
            arrows = self.__arrows
        if to_arrows is None:
            to_arrows = self.__to_arrows
        lat_on_arrow, lng_on_arrow = lat, lng
        if arrow_id is None:
            arrow_id, _, lat_on_arrow, lng_on_arrow, _ = self.get_point_to_arrows(lat, lng, arrows=arrows)
        remaining_distance = self.get_approximate_distance(
            lat_on_arrow, lng_on_arrow, arrows[arrow_id]["lat2"], arrows[arrow_id]["lng2"])
        if delta_distance <= remaining_distance:
            lat12 = arrows[arrow_id]["lat2"] - lat_on_arrow
            lng12 = arrows[arrow_id]["lng2"] - lng_on_arrow
            len12 = math.hypot(lat12, lng12)
            u_lat12 = lat12 / len12
            u_lng12 = lng12 / len12
            advanced_lat = lat_on_arrow + (u_lat12 * delta_distance)
            advanced_lng = lng_on_arrow + (u_lng12 * delta_distance)
            return advanced_lat, advanced_lng
        else:
            next_arrow = arrows[to_arrows[arrow_id][0]]
            return self.get_advanced_latlng(
                next_arrow["lat1"], next_arrow["lng1"], delta_distance - remaining_distance, next_arrow["name"],
                arrows=arrows, to_arrows=to_arrows)

    @staticmethod
    def get_waypoints_names(arrow_id):
        return arrow_id.split("/")

    def get_arrow_ids_to_arrows(self, arrow_ids):
        arrows = {}
        to_arrows = {}
        from_arrows = {}
        for arrow_id in arrow_ids:
            arrows[arrow_id] = self.__arrows[arrow_id]

        for i in range(1, len(arrow_ids)):
            to_arrows[arrow_ids[i - 1]] = [arrow_ids[i]]
            from_arrows[arrow_ids[i]] = [arrow_ids[i - 1]]

        return arrows, to_arrows, from_arrows

    def get_arrow_ids_from_waypoint_id(self, waypoint_id):
        return list(map(lambda x: x[0], filter(lambda x: waypoint_id in x[1]["waypointIDs"], self.__arrows.items())))

    def get_arrow_ids_set_from_waypoint_ids(self, waypoint_ids):
        arrow_ids_set = [[]]
        for waypoint_id in waypoint_ids:
            arrow_ids = self.get_arrow_ids_from_waypoint_id(waypoint_id)
            for arrow_id in arrow_ids:
                if self.__arrows[arrow_id]["waypointIDs"][-1] in waypoint_ids:
                    for i in range(len(arrow_ids_set)):
                        arrow_ids_set[i].append(arrow_id)
        return arrow_ids_set
