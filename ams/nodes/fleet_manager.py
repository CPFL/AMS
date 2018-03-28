#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import Topic, Target, Relation
from ams.nodes import EventLoop
from ams.messages import TrafficSignalStatus, FleetStatus
from ams.structures import FLEET_MANAGER, TRAFFIC_SIGNAL


class FleetManager(EventLoop):

    CONST = FLEET_MANAGER

    def __init__(self, _id, name, waypoint, arrow, route, dt=3.0):
        super().__init__(_id)

        self.status = FleetStatus.new_data(
            name=name,
            time=time(),
            state=FLEET_MANAGER.STATE.LOG_IN,
            relations={}
        )

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.relation = Relation()
        self.traffic_signals = self.manager.dict()
        self.state_machine = None
        self.dt = dt

        self.__pubTopicStatus = Topic()
        self.__pubTopicStatus.set_targets(self.target)
        self.__pubTopicStatus.set_categories(FLEET_MANAGER.TOPIC.CATEGORIES.STATUS)

        self.__topicSubTrafficSignalStatus = Topic()
        self.__topicSubTrafficSignalStatus.set_targets(Target.new_target(None, TRAFFIC_SIGNAL.NODE_NAME), None)
        self.__topicSubTrafficSignalStatus.set_categories(TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS)
        self.__topicSubTrafficSignalStatus.set_message(TrafficSignalStatus)
        self.set_subscriber(self.__topicSubTrafficSignalStatus, self.update_traffic_signal_status)

        self.set_main_loop(self.__main_loop)

    def publish_status(self):
        self.status.relations = dict(map(
            lambda key: (Target.get_code(key), list(map(Target.get_code, self.relation.get_related(key)))),
            self.relation.get_keys()
        ))
        payload = self.__pubTopicStatus.serialize(self.status)
        self.publish(self.__pubTopicStatus, payload)

    def update_traffic_signal_status(self, _client, _userdata, _topic, payload):
        traffic_signal = self.__topicSubTrafficSignalStatus.unserialize(payload)
        self.traffic_signals[traffic_signal["route_code"]] = traffic_signal

    def update_status(self):
        return

    def __main_loop(self):

        while self.status.state != FLEET_MANAGER.STATE.LOG_OUT:
            sleep(self.dt)
            self.update_status()
            self.publish_status()

        return True
