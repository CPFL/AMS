#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams import logger
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
        self.dt = 1.0
        self.initials = {
            "config": None,
            "status": None,
        }
        self.user_data = {
            "clients": {},
            "target_roles": {
                "vehicle": Target.new_target(group, _id)
            }
        }
        self.subscribers = {}

    def __set_vehicle_subscriber(self):
        topic = self.Subscriber.get_vehicle_schedules_topic(self.user_data["target_roles"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_vehicle_schedules_message,
            "structure": self.DispatcherMessage.Schedules,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_transportation_status_topic(self.user_data["target_roles"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_transportation_status_message,
            "structure": self.DispatcherMessage.TransportationStatus,
            "user_data": self.user_data
        }

    def set_rate(self, hz=1.0):
        self.dt = 1.0/hz

    def set_kvs_client(self, kvs_client):
        self.user_data["clients"]["kvs"] = kvs_client

    def set_mqtt_client(self, mqtt_client):
        self.user_data["clients"]["mqtt"] = mqtt_client

    def set_maps_client(self, maps_client):
        self.user_data["clients"]["maps"] = maps_client

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
            logger.info("subscribe: {}".format(subscriber["topic"]))
            self.user_data["clients"]["mqtt"].subscribe(**subscriber)

    def __connect_and_subscribe(self):
        self.user_data["clients"]["kvs"].connect()
        self.subscribe()
        self.user_data["clients"]["mqtt"].connect()

    def start(self):
        self.__set_vehicle_subscriber()

        self.__connect_and_subscribe()

        if self.initials["config"] is not None:
            self.Helper.set_vehicle_config(
                self.user_data["clients"], self.user_data["target_roles"], self.initials["config"])
        if self.initials["status"] is not None:
            self.Helper.set_vehicle_status(
                self.user_data["clients"], self.user_data["target_roles"], self.initials["status"])

        try:
            while True:
                _ = self.StateMachine.update_vehicle_state(
                    clients=self.user_data["clients"],
                    target_roles=self.user_data["target_roles"]
                )

                vehicle_status_key, vehicle_status = self.Helper.get_vehicle_status_key_and_value(
                    self.user_data["clients"], self.user_data["target_roles"])
                vehicle_config_key, vehicle_config = self.Helper.get_vehicle_config_key_and_value(
                    self.user_data["clients"], self.user_data["target_roles"])
                self.user_data["target_roles"]["dispatcher"] = vehicle_config.target_dispatcher
                self.Publisher.publish_vehicle_status(
                    self.user_data["clients"], self.user_data["target_roles"], vehicle_status)

                if vehicle_status.state == self.CONST.STATE.END_PROCESSING:
                    break

                sleep(self.dt)

        except KeyboardInterrupt:
            _, vehicle_status = \
                self.Helper.get_vehicle_status_key_and_value(self.user_data["clients"], self.user_data["target_roles"])
            self.StateMachine.EventHandler.Transition.to_end_processing(vehicle_status)
            self.Helper.set_vehicle_status(self.user_data["clients"], self.user_data["target_roles"], vehicle_status)

        self.user_data["clients"]["mqtt"].disconnect()
