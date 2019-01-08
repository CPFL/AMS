#!/usr/bin/env python
# coding: utf-8

from ams.structures import Location as Structure
from ams.structures import Locations as Structures


class Location(object):

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
