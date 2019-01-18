#!/usr/bin/env python
# coding: utf-8

from ams.structures import Location as Structure
from ams.structures import Locations as Structures


class Location(object):

    Structure = Structure

    @staticmethod
    def new_location(waypoint_id, lane_code, geohash=None):
        return Structure.new_data(
            waypoint_id=waypoint_id,
            lane_code=lane_code,
            geohash=geohash
        )

    validate_location = Structure.validate_data

    @staticmethod
    def new_locations(locations):
        return Structures.new_data(locations)

    @staticmethod
    def same_locations(location1, location2):
        return all([
            location1.waypoint_id == location2.waypoint_id,
            location1.lane_code == location2.lane_code
        ])
