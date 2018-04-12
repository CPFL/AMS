#!/usr/bin/env python
# coding: utf-8

import json
from ams.structures import Location as Structure
from ams.structures import Locations as Structures


class Location(object):

    def __init__(self):
        self.__locations = {}

    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.set_locations(data["locations"])
        return True

    def set_locations(self, locations):
        self.__locations = locations

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
