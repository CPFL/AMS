#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import logger
from ams.helpers import StateMachine as StateMachineHelper
from ams.nodes.base import EventLoop as BaseEventLoop
from ams.nodes.autoware import Message as VehicleMessage
from ams.nodes.sim_autoware import CONST, Structure, Message, Helper, Hook, Condition, Publisher, Subscriber


class EventLoop(BaseEventLoop):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Hook = Hook
    Condition = Condition
    Publisher = Publisher
    Subscriber = Subscriber

    VehicleMessage = VehicleMessage

    def __init__(self, config, status, state_machine_path=None):
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

        self.state_machine_path = state_machine_path

    def loop(self):
        self.user_data["state_machine"] = StateMachineHelper.load(self.state_machine_path)
        self.user_data["state_machine"]["variables"]["clients"] = self.user_data["clients"]
        self.user_data["state_machine"]["variables"]["target_roles"] = self.user_data["target_roles"]
        StateMachineHelper.set_callbacks(
            self.user_data["state_machine"],
            [
                self.Hook.initialize_closest_waypoint,
                self.Hook.initialize_state_cmd,
                self.Hook.initialize_lane_waypoints_array,
                self.Hook.update_closest_waypoint,
                self.Hook.update_current_pose,
                self.Condition.lane_waypoints_array_exists,
                self.Condition.closest_waypoint_initialized,
                self.Condition.state_cmd_is_engage,
                self.Condition.state_cmd_initialized,
                self.Condition.closest_waypoint_is_end_point
            ]
        )

        while True:
            start_time = time()
            self.user_data["state_machine"]["variables"]["config"] = self.Helper.get_config(
                self.user_data["clients"], self.user_data["target_roles"])
            self.user_data["state_machine"]["variables"]["status"] = self.Helper.get_status(
                self.user_data["clients"], self.user_data["target_roles"])
            if StateMachineHelper.update(self.user_data["state_machine"]):
                logger.info(
                    self.Helper.get_status(
                        self.user_data["clients"],
                        self.user_data["target_roles"]).decision_maker_state.data
                )

            self.Helper.set_decision_maker_state(
                self.user_data["clients"], self.user_data["target_roles"],
                self.Structure.Status.DecisionMakerState.new_data(
                    data=StateMachineHelper.get_state(self.user_data["state_machine"])))

            self.Publisher.publish_current_pose(self.user_data["clients"], self.user_data["target_roles"])
            self.Publisher.publish_closest_waypoint(self.user_data["clients"], self.user_data["target_roles"])
            self.Publisher.publish_decision_maker_state(self.user_data["clients"], self.user_data["target_roles"])

            sleep(max(0, self.dt - (time()-start_time)))
