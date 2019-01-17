#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams.helpers import Subscriber
from ams.helpers import StateMachine as StateMachineHelper
from ams.structures import Dispatcher as Structure
from ams.structures import Vehicle
from ams.nodes.event_loop import EventLoop


class Dispatcher(EventLoop):

    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message

    def __init__(self, config, status, state_machine_path=None):
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
                "callback": Subscriber.on_vehicle_status_message,
                "structure": Vehicle.Message.Status,
                "user_data": self.user_data
            }


        self.state_machine_path = state_machine_path

    def loop(self):
        while True:
            sleep(self.dt)
