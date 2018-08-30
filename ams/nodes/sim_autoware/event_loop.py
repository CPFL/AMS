#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams import logger
from ams.nodes.base import EventLoop as BaseEventLoop
from ams.nodes.autoware import Message as VehicleMessage
from ams.nodes.sim_autoware import CONST, Structure, Message, Helper, Publisher, StateMachine, Subscriber


class EventLoop(BaseEventLoop):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine
    Subscriber = Subscriber

    VehicleMessage = VehicleMessage

    def __init__(self, config, status):
        super(EventLoop, self).__init__(config, status)
        self.user_data["target_roles"]["autoware"] = self.user_data["target_roles"]["self"]

        topic = self.Subscriber.get_route_code_message_topic(self.user_data["target_roles"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_route_code_message,
            "structure": self.VehicleMessage.RouteCode,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_state_cmd_topic(self.user_data["target_roles"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_state_cmd,
            "structure": self.Structure.Status.StateCMD,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_light_color_topic(self.user_data["target_roles"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_light_color,
            "structure": self.Structure.Status.LightColor,
            "user_data": self.user_data
        }

    def loop(self):
        while True:
            update_flag = self.StateMachine.update(self.user_data["clients"], self.user_data["target_roles"])
            if update_flag:
                logger.info(
                    self.Helper.get_status(
                        self.user_data["clients"],
                        self.user_data["target_roles"]).decision_maker_state.data
                )

            self.Publisher.publish_current_pose(self.user_data["clients"], self.user_data["target_roles"])
            self.Publisher.publish_closest_waypoint(self.user_data["clients"], self.user_data["target_roles"])
            self.Publisher.publish_decision_maker_state(self.user_data["clients"], self.user_data["target_roles"])

            sleep(self.dt)
