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
    TransportationConfig = Structure.TransportationConfig
    Message = Structure.Message

    def __init__(self, config, status, state_machine_path=None):
        super(Dispatcher, self).__init__(config, status)

        self.user_data["state_machine_path"] = state_machine_path
        self.user_data["target_dispatcher"] = self.config.target_self
        self.user_data["targets"] = self.config.targets

        for target in self.config.target_vehicles:
            topic = Subscriber.get_vehicle_status_topic(target)
            self.subscribers[topic] = {
                "topic": topic,
                "callback": Subscriber.on_vehicle_status_message,
                "structure": Vehicle.Message.Status,
                "user_data": self.user_data
            }

            topic = Subscriber.get_vehicle_config_topic(target)
            self.subscribers[topic] = {
                "topic": topic,
                "callback": Subscriber.on_vehicle_config_message,
                "structure": Vehicle.Message.Config,
                "user_data": self.user_data
            }

        self.state_machine_path = state_machine_path

    def loop(self):
        resource = StateMachineHelper.load_resource(self.state_machine_path),
        self.user_data["state_machine"] = StateMachineHelper.create_data(resource)
        self.user_data["state_machine"]["variables"].update(self.user_data)
        StateMachineHelper.attach(
            self.user_data["state_machine"],
            []
        )
        while True:
            sleep(self.dt)
