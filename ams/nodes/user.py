#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import Topic, Schedule, Target
from ams.nodes import EventLoop
from ams.messages import UserStatus
from ams.structures import Schedules, USER


class User(EventLoop):

    CONST = USER

    def __init__(self, name, dt=1.0):
        super().__init__()

        self.topicStatus = Topic()
        self.topicStatus.set_id(self.event_loop_id)
        self.topicStatus.set_root(USER.TOPIC.PUBLISH)

        self.topicSchedules = Topic()
        self.topicSchedules.set_id(self.event_loop_id)
        self.topicSchedules.set_root(USER.TOPIC.SUBSCRIBE)

        self.name = name
        self.state = USER.STATE.LOG_IN
        self.trip_schedules = None
        self.schedules = None
        self.vehicle_id = None
        self.dt = dt

        self.add_on_message_function(self.update_schedules)
        self.set_subscriber(self.topicSchedules.private+"/schedules")
        self.set_main_loop(self.__main_loop)

    def set_trip_schedules(self, trip_schedules):
        self.trip_schedules = trip_schedules
        self.schedules = [Schedule.new_schedule(
            targets=[Target.new_node_target(self)],
            event=USER.ACTION.REQUEST,
            start_time=trip_schedules[0].period.start,
            end_time=trip_schedules[0].period.end
        )]

    def get_status(self):
        return UserStatus.new_data(
            name=self.name,
            time=time(),
            trip_schedules=self.trip_schedules,
            state=self.state,
            schedule=self.schedules[0]
        )

    def publish_status(self):
        message = self.get_status()
        payload = self.topicStatus.serialize(message)
        self.publish(self.topicStatus.private, payload)

    def update_schedules(self, _client, _userdata, topic, payload):
        if topic == self.topicSchedules.private+"/schedules":
            message = self.topicSchedules.unserialize(payload)
            self.schedules = Schedules.new_data(message)

    def update_status(self):
        return

    def __main_loop(self):

        while self.state != USER.STATE.LOG_OUT:
            sleep(self.dt)
            self.update_status()
            self.publish_status()

        return True
