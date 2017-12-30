#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import Topic
from ams.nodes import EventLoop
from ams.messages import traffic_signal_message


class TrafficSignal(EventLoop):

    class STATE(object):
        GREEN = "green"
        YELLOW = "yellow"
        RED = "red"

    class TOPIC(object):
        PUBLISH = "pub_traffic_signal"
        SUBSCRIBE = "sub_traffic_signal"

    def __init__(self, name, processing_cycle=1.0):
        super().__init__()

        self.trafficSignalPublishTopic = Topic()
        self.trafficSignalPublishTopic.set_id(self.event_loop_id)
        self.trafficSignalPublishTopic.set_root(TrafficSignal.TOPIC.PUBLISH)
        self.trafficSignalPublishTopic.set_message(traffic_signal_message)

        self.trafficSignalSubscribeTopic = Topic()
        self.trafficSignalSubscribeTopic.set_id(self.event_loop_id)
        self.trafficSignalSubscribeTopic.set_root(TrafficSignal.TOPIC.SUBSCRIBE)
        self.trafficSignalSubscribeTopic.set_message(traffic_signal_message)

        self.name = name
        self.routes = {}
        self.schedules = {}
        self.cycles = {}
        self.__processing_cycle = processing_cycle
        self.__check_time = time()
        self.__publish_flag = False

        self.add_on_message_function(self.update_routes)
        self.add_on_message_function(self.update_schedules)
        self.add_on_message_function(self.update_cycles)
        self.set_subscriber(self.trafficSignalSubscribeTopic.private+"/routes")
        self.set_subscriber(self.trafficSignalSubscribeTopic.private+"/schedules")
        self.set_subscriber(self.trafficSignalSubscribeTopic.private+"/cycles")
        self.set_main_loop(self.__main_loop)

    def publish_status(self):
        message = self.trafficSignalPublishTopic.get_template()
        message["name"] = self.name
        message["time"] = time()
        message["routes"] = list(self.routes.values())

        payload = self.trafficSignalPublishTopic.serialize(message)
        self.publish(self.trafficSignalPublishTopic.private, payload)

    def update_routes(self, _client, _userdata, topic, payload):
        if topic == self.trafficSignalSubscribeTopic.private+"/routes":
            routes = self.trafficSignalSubscribeTopic.unserialize(payload)
            self.routes = dict(map(lambda x: (x["route_code"], x), routes))
            self.__publish_flag = True

    def update_schedules(self, _client, _userdata, topic, payload):
        if topic == self.trafficSignalSubscribeTopic.private+"/schedules":
            schedules = self.trafficSignalSubscribeTopic.unserialize(payload)
            self.schedules = dict(map(lambda x: (x["route_code"], [x]), schedules))
            self.__publish_flag = True

    def update_cycles(self, _client, _userdata, topic, payload):
        if topic == self.trafficSignalSubscribeTopic.private+"/cycles":
            cycles = self.trafficSignalSubscribeTopic.unserialize(payload)
            for cycle in cycles:
                for route_code in cycle["route_codes"]:
                    self.cycles[route_code] = cycle
            self.__publish_flag = True

    @staticmethod
    def get_schedule_from_cycle(cycle, start_time):
        base_time = cycle["base_time"]
        period = cycle["period"]
        phase_time = (start_time - base_time) % period
        state = None
        end_time = start_time
        elapse_time = 0
        for phase in cycle["phases"]:
            elapse_time += phase["duration"]
            state = phase["state"]
            end_time = start_time + (elapse_time - phase_time)
            if phase_time < elapse_time:
                break

        schedule = {
            "state": state,
            "start_time": start_time,
            "duration": end_time - start_time,
        }
        return schedule

    def __update_schedules(self):
        current_time = time()
        for route_code, route in self.routes.items():
            route_schedules = []
            if route_code in self.schedules:
                route_schedules = list(
                    filter(lambda x: current_time <= x["start_time"] + x["duration"], self.schedules[route_code]))

            if route_code in self.cycles:
                if len(route_schedules) < 3:
                    if len(route_schedules) == 0:
                        start_time = current_time
                    else:
                        start_time = route_schedules[-1]["start_time"] + route_schedules[-1]["duration"]
                    route_schedules.append(self.get_schedule_from_cycle(self.cycles[route_code], start_time))

            self.schedules[route_code] = route_schedules

    def update_status(self):
        for route_code, route in self.routes.items():
            if len(self.schedules[route_code]) == 0:
                if route["state"] is not None:
                    self.routes[route_code]["state"] = None
                    self.__publish_flag = True
            else:
                current_schedule = self.schedules[route_code][0]
                state = current_schedule["state"]
                if route["state"] != state:
                    self.routes[route_code]["state"] = state
                    self.__publish_flag = True

    def update_check_time(self):
        if len(self.routes) == 0:
            return 1

        self.__check_time = time() + 600
        for route_schedules in self.schedules.values():
            if 0 < len(route_schedules):
                self.__check_time = min(
                    self.__check_time, route_schedules[0]["start_time"] + route_schedules[0]["duration"])

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
