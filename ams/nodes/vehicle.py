#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams.helpers import Subscriber, Hook
from ams.structures import Vehicle as Structure
from ams.structures import AutowareInterface, Autoware, Dispatcher
from ams.nodes.event_loop import EventLoop


class Vehicle(EventLoop):

    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message

    def __init__(self, config, status, state_machine_path=None):
        super(Vehicle, self).__init__(config, status)

        self.user_data["state_machine_path"] = state_machine_path
        self.user_data["target_vehicle"] = self.config.target_self
        self.user_data["target_autoware"] = self.config.target_autoware
        self.user_data["target_dispatcher"] = self.config.target_dispatcher

        topic = Subscriber.get_vehicle_schedule_topic(self.config.target_dispatcher, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_vehicle_schedule_message,
            "structure": Dispatcher.Message.Schedule,
            "user_data": self.user_data
        }

        topic = Subscriber.get_vehicle_event_topic(self.config.target_dispatcher, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_vehicle_event_message,
            "structure": Dispatcher.Message.Event,
            "user_data": self.user_data
        }

        topic = Subscriber.get_stop_signal_topic(self.config.target_dispatcher, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_stop_signal_message,
            "structure": Dispatcher.Message.Signal,
            "user_data": self.user_data
        }

        topic = Subscriber.get_route_point_topic(self.config.target_autoware, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_route_point_message,
            "structure": AutowareInterface.Message.RoutePoint,
            "user_data": self.user_data
        }

        topic = Subscriber.get_current_pose_topic(self.config.target_autoware, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_current_pose,
            "structure": Autoware.ROSMessage.CurrentPose,
            "user_data": self.user_data
        }

        topic = Subscriber.get_decision_maker_state_topic(self.config.target_autoware, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_decision_maker_state_message_update_state,
            "structure": AutowareInterface.Message.DecisionMakerState,
            "user_data": self.user_data
        }

    def loop(self):
        while True:
            status = Hook.get_status(self.user_data["kvs_client"], self.user_data["target_vehicle"], self.Status)
            if status is not None and status.state == Structure.CONST.STATE.END:
                break
            sleep(self.dt)
        if self.user_data["kvs_client"] is not None:
            self.user_data["kvs_client"].disconnect()
        if self.user_data["pubsub_client"] is not None:
            self.user_data["pubsub_client"].disconnect()
