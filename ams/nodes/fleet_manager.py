#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import Topic, Target
from ams.nodes import EventLoop
from ams.messages import TrafficSignalStatus, FleetStatus
from ams.structures import FLEET_MANAGER, TRAFFIC_SIGNAL
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class FleetManager(EventLoop):

    CONST = FLEET_MANAGER

    def __init__(self, _id, name, waypoint, arrow, route, dt=3.0):
        super().__init__(_id)

        self.name = name
        self.state = FLEET_MANAGER.STATE.STAND_BY
        self.dt = dt

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route

        self.traffic_signals = {}
        self.relations = {}  # user_id <-> vehicle_id, route_id <-> vehicle_id

        self.__pubTopicStatus = Topic()
        self.__pubTopicStatus.set_targets(self.target)
        self.__pubTopicStatus.set_categories(FLEET_MANAGER.TOPIC.CATEGORIES.STATUS)

        self.__topicSubTrafficSignalStatus = Topic()
        self.__topicSubTrafficSignalStatus.set_targets(Target.new_target(None, TRAFFIC_SIGNAL.NODE_NAME), None)
        self.__topicSubTrafficSignalStatus.set_categories(TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS)
        self.__topicSubTrafficSignalStatus.set_message(TrafficSignalStatus)
        self.set_subscriber(self.__topicSubTrafficSignalStatus, self.update_traffic_signal_status)

        self.set_main_loop(self.__main_loop)

    def get_status(self):
        return FleetStatus.new_data(
            name=self.name,
            time=time(),
            state=self.state,
            relations=self.relations
        )

    def publish_status(self):
        message = self.get_status()
        payload = self.__pubTopicStatus.serialize(message)
        self.publish(self.__pubTopicStatus, payload)

    def update_traffic_signal_status(self, _client, _userdata, _topic, payload):
        traffic_signal = self.__topicSubTrafficSignalStatus.unserialize(payload)
        self.traffic_signals[traffic_signal["route_code"]] = traffic_signal

    def update_status(self):
        return

    def __main_loop(self):
        sleep(1)

        self.publish_status()

        self.state = FLEET_MANAGER.STATE.RUNNING
        while self.state == FLEET_MANAGER.STATE.RUNNING:
            sleep(self.dt)
            self.update_status()
            self.publish_status()

        return True
