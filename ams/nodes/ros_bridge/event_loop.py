#!/usr/bin/env python
# coding: utf-8

import rospy

from ams import logger
from ams.helpers import Target
from ams.nodes.autoware import Message as VehicleMessage
from ams.nodes.ros_bridge import CONST, Message, Helper, Publisher, Subscriber


class EventLoop(object):

    CONST = CONST
    Message = Message
    Helper = Helper
    Publisher = Publisher
    Subscriber = Subscriber

    VehicleMessage = VehicleMessage

    def __init__(self, _id, group=CONST.NODE_NAME):
        self.dt = 1.0
        self.user_data = {
            "clients": {},
            "target_roles": {
                self.CONST.ROLE_NAME: Target.new_target(group, _id)
            }
        }
        self.subscribers = {}

    def __set_ros_bridge_subscriber(self):
        topic = self.Subscriber.get_route_code_message_topic(self.user_data["target_roles"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_route_code_message,
            "structure": VehicleMessage.RouteCode,
            "user_data": self.user_data
        }

    def set_pubsub_clients(self, aws_iot_client, ros_client):
        self.user_data["clients"]["aws_iot"] = aws_iot_client
        self.user_data["clients"]["ros"] = ros_client

    def set_maps_client(self, maps_client):
        self.user_data["clients"]["maps"] = maps_client

    def subscribe(self):
        for subscriber in self.subscribers.values():
            logger.info("subscribe: {}".format(subscriber["topic"]))
            self.user_data["clients"]["aws_iot"].subscribe(**subscriber)

    def __connect_and_subscribe(self):
        self.user_data["clients"]["ros"].connect()
        self.subscribe()
        self.user_data["clients"]["aws_iot"].connect()

    def start(self):
        self.__set_ros_bridge_subscriber()

        self.__connect_and_subscribe()

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

        self.user_data["clients"]["aws_iot"].disconnect()
