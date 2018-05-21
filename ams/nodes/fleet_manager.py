#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams.helpers import Topic, Target, Relation
from ams.nodes import EventLoop
from ams.messages import TrafficSignalStatus, FleetStatus
from ams.structures import FLEET_MANAGER, TRAFFIC_SIGNAL


class FleetManager(EventLoop):

    CONST = FLEET_MANAGER

    def __init__(self, _id, name, dt=3.0):
        super().__init__(_id)

        self.waypoint = None
        self.arrow = None
        self.route = None

        self.status = FleetStatus.new_data(
            name=name,
            time=time(),
            state=FLEET_MANAGER.STATE.LOG_IN,
            relations={}
        )

        self.relation = Relation()
        self.traffic_signals = self.manager.dict()
        self.state_machine = None
        self.dt = dt

        self.__pub_status_topic = Topic.get_topic(
            from_target=self.target,
            categories=FLEET_MANAGER.TOPIC.CATEGORIES.STATUS
        )

        self.set_subscriber(
            topic=Topic.get_topic(
                from_target=Target.new_target(TRAFFIC_SIGNAL.NODE_NAME, None),
                categories=TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS,
                use_wild_card=True
            ),
            callback=self.update_traffic_signal_status,
            structure=TrafficSignalStatus
        )

    def set_maps(self, waypoint, arrow, route):
        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route

    def publish_status(self):
        self.status.relations = dict(map(
            lambda key: (Target.get_code(key), list(map(Target.get_code, self.relation.get_related(key)))),
            self.relation.get_keys()
        ))
        payload = Topic.serialize(self.status)
        self.publish(self.__pub_status_topic, payload)

    def update_traffic_signal_status(self, _client, _userdata, _topic, traffic_signal):
        self.traffic_signals[traffic_signal.route_code] = traffic_signal

    def update_status(self):
        return
