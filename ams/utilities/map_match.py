#!/usr/bin/env python
# coding: utf-8

from ams import Route, Location


class MapMatch(object):
    def __init__(self):
        self.__waypoint = None
        self.__arrow = None
        self.__route = Route()

    def set_waypoint(self, waypoint):
        self.__waypoint = waypoint
        self.__route.set_waypoint(self.__waypoint)

    def set_arrow(self, arrow):
        self.__arrow = arrow
        self.__route.set_arrow(self.__arrow)

    @staticmethod
    def get_similarity_between_poses(pose1, pose2):
        similarity = 1.0
        print(pose1, pose2)
        return similarity

    def get_matched_location_on_route(self, pose, route):
        locations = self.__route.get_locations(route)
        similarity_max = 0.0
        matched_location = None
        for location in locations:
            pose_on_route = self.__waypoint.get_pose(location.waypoint_id)
            similarity = MapMatch.get_similarity_between_poses(pose, pose_on_route)
            if matched_location is None or similarity_max < similarity:
                matched_location = location
                similarity_max = similarity
        return matched_location

    def get_matched_location_on_arrows(self, pose, arrow_codes=None):
        if arrow_codes is None:
            arrow_codes = self.__arrow.get_arrow_codes()

        similarity_max = 0.0
        matched_location = None
        for arrow_code in arrow_codes:
            for waypoint_id in self.__arrow.get_waypoint_ids(arrow_code):
                pose_on_route = self.__waypoint.get_pose(waypoint_id)
                similarity = MapMatch.get_similarity_between_poses(pose, pose_on_route)
                if matched_location is None or similarity_max < similarity:
                    matched_location = Location.new_location(waypoint_id, arrow_code)
                    similarity_max = similarity
        return matched_location
