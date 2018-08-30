#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams.nodes.base import CONST, Structure, Message, Helper, Subscriber


class EventLoop(object):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Subscriber = Subscriber

    def __init__(self, config, status):
        self.dt = 1.0
        self.config = self.Structure.Config.new_data(**config)
        self.status = self.Structure.Status.new_data(**status)
        self.user_data = {
            "clients": {},
            "target_roles": self.config.target_roles
        }
        self.subscribers = {}

        topic = self.Subscriber.get_request_get_config_topic(self.config.target_roles)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_request_get_config_message,
            "structure": self.Message.RequestConfig,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_request_get_status_topic(self.config.target_roles)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_request_get_status_message,
            "structure": self.Message.RequestStatus,
            "user_data": self.user_data
        }

    def set_clients(self, kvs_client, pubsub_client, maps_client=None):
        self.user_data["clients"]["kvs"] = kvs_client
        if pubsub_client is not None:
            self.user_data["clients"]["pubsub"] = pubsub_client
        if maps_client is not None:
            self.user_data["clients"]["maps"] = maps_client

    def set_rate(self, hz=1.0):
        self.dt = 1.0/hz

    def connect_clients(self):
        self.user_data["clients"]["kvs"].connect()
        self.Helper.set_config(self.user_data["clients"], self.config.target_roles, self.config)
        self.Helper.set_status(self.user_data["clients"], self.config.target_roles, self.status)

        for subscriber in self.subscribers.values():
            self.user_data["clients"]["pubsub"].subscribe(**subscriber)
        self.user_data["clients"]["pubsub"].connect()

    def loop(self):
        while True:
            sleep(self.dt)

    def start(self):
        self.connect_clients()

        try:
            self.loop()

        except KeyboardInterrupt:
            self.user_data["clients"]["pubsub"].disconnect()
