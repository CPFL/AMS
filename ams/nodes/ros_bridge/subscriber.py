#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.ros_bridge import Helper, Publisher
from ams.nodes.autoware import CONST as AUTOWARE


class Subscriber(object):

    Helper = Helper
    Publisher = Publisher

    VEHICLE = AUTOWARE

    @classmethod
    def get_route_code_message_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["vehicle"],
            to_target=target_roles["ros"],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.ROUTE_CODE
        )

    @classmethod
    def on_route_code_message(cls, _client, user_data, _topic, route_code_message):
        lane_array = cls.Helper.get_lane_array(user_data["clients"], route_code_message.body)
        cls.Publisher.publish_lane_array(user_data["clients"], lane_array)
