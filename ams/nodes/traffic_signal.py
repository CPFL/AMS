#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams.helpers import Topic, Schedule
from ams.nodes import EventLoop
from ams.messages import TrafficSignalStatus as Status
from ams.structures import Cycle, Schedules, TRAFFIC_SIGNAL


class TrafficSignal(EventLoop):

    CONST = TRAFFIC_SIGNAL

    def __init__(self, _id, route_code, state=TRAFFIC_SIGNAL.STATE.UNKNOWN, processing_cycle=1.0):
        super().__init__(_id)

        self.route_code = route_code
        self.state = state

        self.schedules = []
        self.cycle = None
        self.__processing_cycle = processing_cycle
        self.__check_time = time()
        self.__publish_flag = False

        self.__pub_status_topic = Topic.get_topic(
            from_target=self.target,
            categories=TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS
        )

        self.set_subscriber(
            topic=Topic.get_topic(
                from_target=None,
                to_target=self.target,
                categories=TRAFFIC_SIGNAL.TOPIC.CATEGORIES.SCHEDULES,
                use_wild_card=True
            ),
            callback=self.update_schedules,
            structure=Schedules
        )

        self.set_subscriber(
            topic=Topic.get_topic(
                from_target=None,
                to_target=self.target,
                categories=TRAFFIC_SIGNAL.TOPIC.CATEGORIES.CYCLE,
                use_wild_card=True
            ),
            callback=self.update_cycle,
            structure=Cycle
        )

        self.set_main_loop(self.__main_loop)

    @staticmethod
    def get_status(route_code, state):
        return Status.new_data(
            route_code=route_code,
            time=time(),
            state=state
        )

    def publish_status(self):
        status = TrafficSignal.get_status(self.route_code, self.state)
        payload = Topic.serialize(status)
        self.publish(self.__pub_status_topic, payload)

    def update_schedules(self, _client, _userdata, _topic, schedules):
        self.schedules = schedules
        self.__publish_flag = True

    def update_cycle(self, _client, _userdata, _topic, cycle):
        self.cycle = cycle
        self.__publish_flag = True

    def __update_schedules(self):
        current_time = time()
        schedules = []
        if self.route_code in self.schedules:
            schedules = list(filter(lambda x: current_time <= x.period.end, self.schedules))

        if self.cycle is not None:
            if len(schedules) < 3:
                if len(schedules) == 0:
                    start_time = current_time
                else:
                    start_time = schedules[-1].period.end
                schedules.append(Schedule.get_schedule_from_cycle([self.target], self.cycle, start_time))
        self.schedules = schedules

    def update_status(self):
        if len(self.schedules) == 0:
            if self.state is not None:
                self.state = TRAFFIC_SIGNAL.STATE.UNKNOWN
                self.__publish_flag = True
        else:
            state = self.schedules[0].event
            if self.state != state:
                self.state = state
                self.__publish_flag = True

    def update_check_time(self):
        self.__check_time = time() + TRAFFIC_SIGNAL.LOWER_LIMIT_RATE
        if 0 < len(self.schedules):
            self.__check_time = min(self.__check_time, self.schedules[0].period.end)

    def __main_loop(self):
        while True:
            if self.__check_time <= time():
                self.__update_schedules()
                self.update_status()
                self.update_check_time()

            if self.__publish_flag:
                self.publish_status()
                self.__check_time = time()
                self.__publish_flag = False

            sleep(self.__processing_cycle)
