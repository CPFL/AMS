#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams import logger
from ams.helpers import Hook, Subscriber
from ams.structures import EventLoop as Structure


class EventLoop(object):

    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message

    def __init__(self, config, status=None):
        self.dt = 1.0
        self.config = self.Config.new_data(**config)
        self.status = self.Status.new_data(**status) if status is not None else None
        self.user_data = {
            "kvs_client": None,
            "pubsub_client": None,
            "maps_client": None,
            "ros_client": None
        }
        self.subscribers = {}
        self.ros_subscribers = {}

        topic = Subscriber.get_request_get_config_topic(self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_request_get_config_message,
            "structure": self.Message.RequestConfig,
            "user_data": self.user_data
        }

        topic = Subscriber.get_request_get_status_topic(self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_request_get_status_message,
            "structure": self.Message.RequestStatus,
            "user_data": self.user_data
        }

    def set_clients(self, kvs_client, pubsub_client, maps_client=None, ros_client=None):
        self.user_data["kvs_client"] = kvs_client
        if pubsub_client is not None:
            self.user_data["pubsub_client"] = pubsub_client
        if maps_client is not None:
            self.user_data["maps_client"] = maps_client
        if ros_client is not None:
            self.user_data["ros_client"] = ros_client

    def set_rate(self, hz=1.0):
        self.dt = 1.0/hz

    def initialize_kvs(self):
        Hook.set_config(self.user_data["kvs_client"], self.config.target_self, self.config)
        if self.status is not None:
            Hook.set_status(self.user_data["kvs_client"], self.config.target_self, self.status)

    def connect_clients(self):
        self.user_data["kvs_client"].connect()
        logger.info(self.config.target_self)
        self.initialize_kvs()

        for subscriber in self.subscribers.values():
            logger.info("subscribe: {}".format(subscriber["topic"]))
            self.user_data["pubsub_client"].subscribe(**subscriber)
        self.user_data["pubsub_client"].connect()

        for ros_subscriber in self.ros_subscribers.values():
            logger.info("subscribe: {}".format(ros_subscriber["topic"]))
            self.user_data["ros_client"].subscribe(**ros_subscriber)
        if self.user_data["ros_client"] is not None:
            self.user_data["ros_client"].connect()

    def loop(self):
        while True:
            sleep(self.dt)

    def start(self):
        self.connect_clients()

        try:
            self.loop()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            logger.error(e)
        finally:
            self.disconnect_clients()

    def disconnect_clients(self):
        if self.user_data["pubsub_client"] is not None:
            self.user_data["pubsub_client"].disconnect()
        if self.user_data["kvs_client"] is not None:
            self.user_data["kvs_client"].disconnect()
        if self.user_data["ros_client"] is not None:
            self.user_data["ros_client"].disconnect()
        if self.user_data["maps_client"] is not None:
            self.user_data["maps_client"].disconnect()
