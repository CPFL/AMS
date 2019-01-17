#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams.helpers import Subscriber, Hook, Target
from ams.structures import Dispatcher as Structure
from ams.structures import Vehicle, TrafficSignal, User
from ams.nodes.event_loop import EventLoop


class Dispatcher(EventLoop):

    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message

    def __init__(self, config, status, state_machine_path=None, owner_id="*"):
        super(Dispatcher, self).__init__(config, status)

        self.user_data["state_machine_path"] = state_machine_path
        self.user_data["target_dispatcher"] = self.config.target_self

        topic = Subscriber.get_dispatcher_event_topic(Target.new_target("owner", owner_id), self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_dispatcher_event_message,
            "structure": Dispatcher.Message.Event,
            "user_data": self.user_data
        }

        for target_vehicle in self.config.target_vehicles:
            topic = Subscriber.get_vehicle_status_topic(target_vehicle, self.config.target_self)
            self.subscribers[topic] = {
                "topic": topic,
                "callback": Subscriber.on_vehicle_status_message_update_dispatcher_state,
                "structure": Vehicle.Message.Status,
                "user_data": self.user_data
            }

        topic = Subscriber.get_traffic_signal_status_topic(Target.new_target(TrafficSignal.CONST.NODE_NAME))
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_traffic_signal_status_message,
            "structure": TrafficSignal.Message.Status,
            "user_data": self.user_data
        }

        if "target_users" in self.config:
            for target_user in self.config.target_users:
                topic = Subscriber.get_user_status_topic(target_user, self.config.target_self)
                self.subscribers[topic] = {
                    "topic": topic,
                    "callback": Subscriber.on_user_status_message_update_user_statuses,
                    "structure": User.Message.Status,
                    "user_data": self.user_data
                }

    def initialize_kvs(self):
        Hook.set_config(self.user_data["kvs_client"], self.config.target_self, self.config)
        if self.status is not None:
            for target_vehicle in self.config.target_vehicles:
                Hook.set_status(
                    self.user_data["kvs_client"], self.config.target_self, self.status, sub_target=target_vehicle)
                if "user_statuses" in self.status:
                    Hook.set_user_statuses(
                        self.user_data["kvs_client"], self.config.target_self, target_vehicle, [])
                if "traffic_signal_statuses" in self.status:
                    Hook.set_traffic_signal_statuses(
                        self.user_data["kvs_client"], self.config.target_self, target_vehicle, [])
            if "target_users" in self.config:
                for target_user in self.config.target_users:
                    Hook.initialize_status(
                        self.user_data["kvs_client"], self.config.target_self, sub_target=target_user)

    def loop(self):
        while True:
            status = Hook.get_status(self.user_data["kvs_client"], self.user_data["target_dispatcher"], self.Status)
            if status is not None and status.state == Structure.CONST.STATE.END:
                break
            sleep(self.dt)
        if self.user_data["kvs_client"] is not None:
            self.user_data["kvs_client"].disconnect()
        if self.user_data["pubsub_client"] is not None:
            self.user_data["pubsub_client"].disconnect()
