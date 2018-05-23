#!/usr/bin/env python
# coding: utf-8

from time import time
from multiprocessing import Manager

from ams import AttrDict
from ams.helpers import Topic, Target, Relation
from ams.messages import FleetStatus
from ams.structures import FLEET_MANAGER


class FleetManager(object):

    CONST = FLEET_MANAGER

    def __init__(self, _id, name):
        self.manager = Manager()
        self.target = Target.new_target(self.__class__.__name__, _id)

        self.mqtt_client = None
        self.status = AttrDict()
        self.maps = AttrDict()

        self.subscribers = []
        self.initialize_fleet_manager(name)

    def initialize_fleet_manager(self, name):
        self.status.relation = Relation()

        self.status.status = FleetStatus.new_data(
            name=name,
            time=time(),
            state=FLEET_MANAGER.STATE.LOG_IN,
            relations={}
        )

        self.status.pub_fleet_manager_status_topic = Topic.get_topic(
            from_target=self.target,
            categories=FLEET_MANAGER.TOPIC.CATEGORIES.STATUS
        )

    def set_mqtt_client(self, mqtt_client):
        self.mqtt_client = mqtt_client

    def set_maps(self, waypoint, arrow, route):
        self.maps.waypoint = waypoint
        self.maps.arrow = arrow
        self.maps.route = route

    def publish_status(self):
        self.status.status.relations = dict(map(
            lambda key: (Target.get_code(key), list(map(Target.get_code, self.status.relation.get_related(key)))),
            self.status.relation.get_keys()
        ))
        payload = Topic.serialize(self.status.status)
        self.mqtt_client.publish(self.status.pub_fleet_manager_status_topic, payload)

    def update_status(self):
        return

    def start(self):
        for subscriber in self.subscribers:
            self.mqtt_client.subscribe(**subscriber)
        self.mqtt_client.connect()

    def stop(self):
        self.mqtt_client.disconnect()