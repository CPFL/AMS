#!/usr/bin/env python
# coding: utf-8

import json
from ams.helpers import Location, Target
from ams.structures import Spot as Structure


class Spot(object):

    def __init__(self):
        self.__spots = {}

    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.set_spots(data["spots"])
        return True

    def set_spots(self, spots):
        self.__spots = dict(map(
            lambda x: (x["ID"], Spot.new_spot(
                Target.new_targets(x["targets"]),
                Location.new_location(
                    x["contact"]["waypointID"], x["contact"]["laneCode"],  x["contact"]["geohash"]
                ),
                None, None
            )),
            spots
        ))

    @staticmethod
    def new_spot(targets, contact, polygon, events):
        return Structure.new_data(
            targets=targets,
            contact=contact,
            polygon=polygon,
            events=events
        )

    def update_spot_events(self, spot_id, events):
        self.__spots[spot_id].events = events

    validate_spot = Structure.validate_data
    get_errors = Structure.get_errors

    def get_spot_ids(self):
        return list(self.__spots.keys())

    def get_spot(self, spot_id):
        return self.__spots[spot_id]

    def get_spots_of_target_group(self, target):
        return dict(filter(lambda x: target in x[1].targets, self.__spots.items()))

    def get_contact(self, spot_id):
        return self.__spots[spot_id].contact

    def get_polygon(self, spot_id):
        return self.__spots[spot_id].polygon

    def get_events(self, spot_id):
        return self.__spots[spot_id].events
