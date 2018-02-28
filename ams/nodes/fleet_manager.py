#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import Topic
from ams.nodes import EventLoop, TrafficSignal
from ams.messages import TrafficSignalStatus, FleetStatus
from ams.structures import FLEET_MANAGER
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class FleetManager(EventLoop):

    CONST = FLEET_MANAGER

    def __init__(self, name, waypoint, arrow, route, dt=3.0):
        super().__init__()

        self.topicTrafficSignalStatus = Topic()
        self.topicTrafficSignalStatus.set_root(TrafficSignal.CONST.TOPIC.PUBLISH)

        self.topicStatus = Topic()
        self.topicStatus.set_id(self.event_loop_id)
        self.topicStatus.set_root(FLEET_MANAGER.TOPIC.PUBLISH)

        self.name = name
        self.state = FLEET_MANAGER.STATE.STAND_BY
        self.dt = dt

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route

        self.traffic_signals = {}
        self.relations = {}  # user_id <-> vehicle_id, route_id <-> vehicle_id

        self.set_subscriber(self.topicTrafficSignalStatus.all, self.update_traffic_signal_status)
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
        payload = self.topicStatus.serialize(message)
        self.publish(self.topicStatus.private, payload)

    def update_traffic_signal_status(self, _client, _userdata, topic, payload):
        if self.topicTrafficSignalStatus.root in topic:
            traffic_signal = TrafficSignalStatus.new_data(**self.topicTrafficSignalStatus.unserialize(payload))
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
