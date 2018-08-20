#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams import logger
from ams.helpers import Target
from ams.nodes.sim_autoware import CONST, Structure, Message, Helper, Publisher, StateMachine, Subscriber


class EventLoop(object):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine
    Subscriber = Subscriber

    def __init__(self, _id, group=CONST.NODE_NAME):
        self.dt = 1.0
        self.initials = {
            "config": None,
            "status": None,
        }
        self.user_data = {
            "clients": {},
            "target_roles": {
                "autoware": Target.new_target(group, _id)
            }
        }
        self.subscribers = {}

    def __set_sim_autoware_subscriber(self):
        topic = self.Subscriber.get_lane_waypoints_array_topic(self.user_data["target_roles"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_lane_waypoints_array,
            "structure": self.Structure.Status.LaneArray,
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

    def set_kvs_client(self, kvs_client):
        self.user_data["clients"]["kvs"] = kvs_client

    def set_pubsub_client(self, pubsub_client):
        self.user_data["clients"]["pubsub"] = pubsub_client

    def set_initial_config(self, target_vehicle):
        self.initials["config"] = self.Structure.Config.new_data(
            target_vehicle=target_vehicle
        )
        self.user_data["target_roles"]["vehicle"] = target_vehicle

    def set_initial_status(self, decision_maker_state, pose=None):
        current_pose = None
        if pose is not None:
            current_pose = self.Structure.Status.CurrentPose.new_data(
                header={
                    "seq": 0,
                    "frame_id": "",
                    "stamp": self.Helper.get_timestamp()
                },
                pose=pose
            )
        self.initials["status"] = self.Structure.Status.new_data(
            decision_maker_state={"data": decision_maker_state},
            current_pose=current_pose,
            closest_waypoint=None,
            lane_waypoints_array=None,
            state_cmd=None,
            light_color=None
        )

    def subscribe(self):
        for subscriber in self.subscribers.values():
            logger.info("subscribe: {}".format(subscriber["topic"]))
            self.user_data["clients"]["pubsub"].subscribe(**subscriber)

    def __connect_and_subscribe(self):
        self.user_data["clients"]["kvs"].connect()
        self.subscribe()
        self.user_data["clients"]["pubsub"].connect()

    def start(self):
        self.__set_sim_autoware_subscriber()

        self.__connect_and_subscribe()

        self.Helper.set_config(
            self.user_data["clients"], self.user_data["target_roles"], self.initials["config"])
        self.Helper.set_status(
            self.user_data["clients"], self.user_data["target_roles"], self.initials["status"])

        try:
            while True:
                _ = self.StateMachine.update(self.user_data["clients"], self.user_data["target_roles"])

                self.Publisher.publish_current_pose(self.user_data["clients"], self.user_data["target_roles"])
                self.Publisher.publish_closest_waypoint(self.user_data["clients"], self.user_data["target_roles"])
                self.Publisher.publish_decision_maker_state(self.user_data["clients"], self.user_data["target_roles"])

                sleep(self.dt)

        except KeyboardInterrupt:
            pass

        self.user_data["clients"]["pubsub"].disconnect()
