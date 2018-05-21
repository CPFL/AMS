#!/usr/bin/env python
# coding: utf-8

from time import time

from ams import AttrDict
from ams.helpers import Topic, Target, Relation
from ams.nodes import EventLoop
from ams.messages import FleetStatus
from ams.structures import FLEET_MANAGER


class FleetManager(EventLoop):

    CONST = FLEET_MANAGER

    def __init__(self, _id, name):
        super().__init__(_id)

        self.cache = AttrDict()
        self.maps = AttrDict()

        self.initialize_fleet_manager(name)

    def initialize_fleet_manager(self, name):
        self.cache.relation = Relation()

        self.cache.status = FleetStatus.new_data(
            name=name,
            time=time(),
            state=FLEET_MANAGER.STATE.LOG_IN,
            relations={}
        )

        self.cache.pub_fleet_manager_status_topic = Topic.get_topic(
            from_target=self.target,
            categories=FLEET_MANAGER.TOPIC.CATEGORIES.STATUS
        )

    def set_maps(self, waypoint, arrow, route):
        self.maps.waypoint = waypoint
        self.maps.arrow = arrow
        self.maps.route = route

    def publish_status(self):
        self.cache.status.relations = dict(map(
            lambda key: (Target.get_code(key), list(map(Target.get_code, self.cache.relation.get_related(key)))),
            self.cache.relation.get_keys()
        ))
        payload = Topic.serialize(self.cache.status)
        self.publish(self.cache.pub_fleet_manager_status_topic, payload)

    def update_status(self):
        return
