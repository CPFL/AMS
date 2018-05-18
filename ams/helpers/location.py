#!/usr/bin/env python
# coding: utf-8

from ams.structures import Location as Structure
from ams.structures import Locations as Structures


class Location(object):

    @staticmethod
    def new_location(waypoint_id, arrow_code, geohash=None):
        return Structure.new_data(
            waypoint_id=waypoint_id,
            arrow_code=arrow_code,
            geohash=geohash
        )

    validate_location = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def new_locations(locations):
        return Structures.new_data(locations)
