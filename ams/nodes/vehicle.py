#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Subscriber
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

        topic = Subscriber.get_vehicle_events_topic(self.config.target_dispatcher, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_vehicle_events_message,
            "structure": Dispatcher.Message.Events,
            "user_data": self.user_data
        }

        topic = Subscriber.get_transportation_status_topic(self.config.target_dispatcher, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_transportation_status_message,
            "structure": Dispatcher.Message.TransportationStatus,
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
            "callback": Subscriber.on_decision_maker_state_update_state,
            "structure": Autoware.ROSMessage.DecisionMakerState,
            "user_data": self.user_data
        }
