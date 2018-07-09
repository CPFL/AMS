#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Target
from ams.nodes.vehicle import Message as VehicleMessage
from ams.nodes.dispatcher import CONST, Structure, Message, Helper, Publisher, StateMachine, Subscriber


class EventLoop(object):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine
    Subscriber = Subscriber

    VehicleMessage = VehicleMessage

    def __init__(self, _id, group=CONST.NODE_NAME):
        self.target = Target.new_target(group, _id)

        self.dt = 1.0
        self.initials = {
            "config": Structure.Config.new_data(targets=[], active_api_keys=[], inactive_api_keys=[]),
            "state": self.CONST.STATE.START_PROCESSING,
            "schedules": None
        }
        self.user_data = {
            "target": self.target,
            "kvs_client": None,
            "mqtt_client": None,
            "maps_client": None,
        }
        self.subscribers = {}

    def __set_dispatcher_subscriber(self):
        for target_vehicle in self.initials["config"].targets:
            topic = self.Subscriber.get_vehicle_status_topic(target_vehicle)
            self.subscribers[topic] = {
                "topic": topic,
                "callback": self.Subscriber.on_vehicle_status_message,
                "structure": self.VehicleMessage.Status,
                "user_data": self.user_data
            }

            topic = self.Subscriber.get_vehicle_config_topic(target_vehicle)
            self.subscribers[topic] = {
                "topic": topic,
                "callback": self.Subscriber.on_vehicle_config_message,
                "structure": self.VehicleMessage.Config,
                "user_data": self.user_data
            }

    def set_kvs_client(self, kvs_client):
        self.user_data["kvs_client"] = kvs_client

    def set_mqtt_client(self, mqtt_client):
        self.user_data["mqtt_client"] = mqtt_client

    def set_maps_client(self, maps_client):
        self.user_data["maps_client"] = maps_client

    def set_initial_config(self, targets, active_api_keys=None, inactive_api_keys=None):
        self.initials["config"] = Structure.Config.new_data(
            targets=targets,
            active_api_keys=active_api_keys if active_api_keys is not None else [],
            inactive_api_keys=inactive_api_keys if inactive_api_keys is not None else []
        )

    def set_initial_state(self, state=CONST.STATE.START_PROCESSING):
        self.initials["state"] = self.Structure.Status.new_data(
            state=state
        )

    def subscribe(self):
        for subscriber in self.subscribers.values():
            print("subscribe: {}".format(subscriber["topic"]))
            self.user_data["mqtt_client"].subscribe(**subscriber)

    def __connect_and_set_user_data(self):
        self.user_data["kvs_client"].connect()
        self.subscribe()
        self.user_data["mqtt_client"].connect()

    def start(self):
        self.__set_dispatcher_subscriber()

        self.__connect_and_set_user_data()

        self.Helper.set_dispatcher_config(self.user_data["kvs_client"], self.target, self.initials["config"])
        self.Helper.set_dispatcher_state(self.user_data["kvs_client"], self.target, self.initials["state"])

    def stop(self):
        self.user_data["mqtt_client"].disconnect()
