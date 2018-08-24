#!/usr/bin/env python
# coding: utf-8

from ams import VERSION
from ams.helpers import Topic, Schedule
from ams.nodes.vehicle import Publisher as VehiclePublisher
from ams.nodes.autoware import CONST, Message


class Publisher(VehiclePublisher):

    VEHICLE = CONST
    Message = Message

    @classmethod
    def get_route_code_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROLE_NAME],
            to_target=target_roles[cls.VEHICLE.ROS.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.ROUTE_CODE,
        )

    @classmethod
    def get_state_cmd_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROLE_NAME],
            to_target=target_roles[cls.VEHICLE.ROS.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.STATE_CMD,
        )

    @classmethod
    def publish_route_code(cls, clients, target_roles, route_code):
        topic = cls.get_route_code_topic(target_roles)
        message = cls.Message.RouteCode.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION
            },
            "body": route_code
        })
        clients["pubsub"].publish(topic, message)

    @classmethod
    def publish_state_cmd(cls, clients, target_roles, state_cmd):
        topic = cls.get_state_cmd_topic(target_roles)
        clients["pubsub"].publish(topic, state_cmd)
