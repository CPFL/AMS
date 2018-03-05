#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import Topic, Schedule
from ams.nodes import EventLoop
from ams.messages import UserStatus
from ams.structures import Schedules, USER, FLEET_MANAGER


class User(EventLoop):

    CONST = USER

    def __init__(self, _id, name, dt=1.0):
        super().__init__(_id)

        self.name = name
        self.state = USER.STATE.LOG_IN
        self.trip_schedules = None
        self.schedules = None
        self.vehicle_id = None
        self.dt = dt

        self.__topicPubStatus = Topic()
        self.__topicPubStatus.set_targets(self.target)
        self.__topicPubStatus.set_categories(USER.TOPIC.CATEGORIES.STATUS)

        self.__topicSubSchedules = Topic()
        self.__topicSubSchedules.set_targets(None, self.target)
        self.__topicSubSchedules.set_categories(FLEET_MANAGER.TOPIC.CATEGORIES.SCHEDULES)
        self.__topicSubSchedules.set_message(Schedules)
        self.set_subscriber(self.__topicSubSchedules, self.update_schedules)

        self.set_main_loop(self.__main_loop)

    def set_trip_schedules(self, trip_schedules):
        self.trip_schedules = trip_schedules
        self.schedules = [Schedule.new_schedule(
            targets=[self.target],
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
        payload = self.__topicPubStatus.serialize(message)
        self.publish(self.__topicPubStatus, payload)

    def update_schedules(self, _client, _userdata, _topic, payload):
        self.schedules = self.__topicSubSchedules.unserialize(payload)

    def update_status(self):
        return

    def __main_loop(self):

        while self.state != USER.STATE.LOG_OUT:
            sleep(self.dt)
            self.update_status()
            self.publish_status()

        return True
