#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams.helpers import Target
from ams.nodes.dispatcher import Message as DispatcherMessage
from ams.nodes.vehicle import CONST, Structure, Message, Helper, Publisher, StateMachine, Subscriber


class EventLoop(object):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine
    Subscriber = Subscriber

    DispatcherMessage = DispatcherMessage

    def __init__(self, _id, group=CONST.NODE_NAME):
        self.target = Target.new_target(group, _id)

        self.dt = 1.0
        self.initials = {
            "config": None,
            "status": None,
        }
        self.user_data = {
            "target": self.target,
            "kvs_client": None,
            "mqtt_client": None,
            "maps_client": None
        }
        self.subscribers = {}

    def __set_vehicle_subscriber(self):
        topic = self.Subscriber.get_vehicle_schedules_topic(self.target)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_vehicle_schedules_message,
            "structure": self.DispatcherMessage.Schedules,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_transportation_status_topic(self.target)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_transportation_status_message,
            "structure": self.DispatcherMessage.TransportationStatus,
            "user_data": self.user_data
        }

    def set_rate(self, hz=1.0):
        self.dt = 1.0/hz

    def set_kvs_client(self, kvs_client):
        self.user_data["kvs_client"] = kvs_client

    def set_mqtt_client(self, mqtt_client):
        self.user_data["mqtt_client"] = mqtt_client

    def set_maps_client(self, maps_client):
        self.user_data["maps_client"] = maps_client

    def set_initial_config(
            self, activation):
        self.initials["config"] = self.Structure.Config.new_data(
            target_dispatcher=None,
            activation=activation
        )

    def set_initial_status(
            self, state=CONST.STATE.START_PROCESSING, schedule_id=None,
            location=None, pose=None, velocity=0.0):
        self.initials["status"] = self.Structure.Status.new_data(
            state=state,
            schedule_id=schedule_id,
            location=location,
            pose=pose,
            velocity=velocity,
            updated_at=self.Helper.get_current_time()
        )

    def subscribe(self):
        for subscriber in self.subscribers.values():
            print("subscribe: {}".format(subscriber["topic"]))
            self.user_data["mqtt_client"].subscribe(**subscriber)

    def __connect_and_subscribe(self):
        self.user_data["kvs_client"].connect()
        self.subscribe()
        self.user_data["mqtt_client"].connect()

    def start(self):
        self.__set_vehicle_subscriber()

        self.__connect_and_subscribe()

        if self.initials["config"] is not None:
            self.Helper.set_vehicle_config(self.user_data["kvs_client"], self.target, self.initials["config"])
        if self.initials["status"] is not None:
            self.Helper.set_vehicle_status(self.user_data["kvs_client"], self.target, self.initials["status"])

        try:
            while True:
                _ = self.StateMachine.update_vehicle_state(
                    target_vehicle=self.target,
                    kvs_client=self.user_data["kvs_client"],
                    mqtt_client=self.user_data["mqtt_client"]
                )

                vehicle_status_key, vehicle_status = \
                    self.Helper.get_vehicle_status_key_and_value(self.user_data["kvs_client"], self.target)
                vehicle_config_key, vehicle_config = \
                    self.Helper.get_vehicle_config_key_and_value(self.user_data["kvs_client"], self.target)
                self.Publisher.publish_vehicle_status(
                    self.user_data["mqtt_client"], self.target, vehicle_status, vehicle_config.target_dispatcher)

                if vehicle_status.state == self.CONST.STATE.END_PROCESSING:
                    break

                sleep(self.dt)

        except KeyboardInterrupt:
            _, vehicle_status = \
                self.Helper.get_vehicle_status_key_and_value(self.user_data["kvs_client"], self.target)
            self.StateMachine.EventHandler.Transition.to_end_processing(vehicle_status)
            self.Helper.set_vehicle_status(self.user_data["kvs_client"], self.target, vehicle_status)

        self.user_data["mqtt_client"].disconnect()
