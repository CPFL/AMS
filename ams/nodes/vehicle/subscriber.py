#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic, Target
from ams.nodes.dispatcher import CONST as DISPATCHER
from ams.nodes.vehicle import Helper, Publisher


class Subscriber(object):

    DISPATCHER = DISPATCHER

    Helper = Helper
    Publisher = Publisher

    @classmethod
    def get_transportation_status_topic(cls, target):
        return Topic.get_topic(
            from_target=Target.new_target(group=cls.DISPATCHER.NODE_NAME),
            to_target=target,
            categories=cls.DISPATCHER.TOPIC.CATEGORIES.TRANSPORTATION_STATUS,
            use_wild_card=True
        )

    @classmethod
    def get_vehicle_schedules_topic(cls, target):
        return Topic.get_topic(
            from_target=None,
            to_target=target,
            categories=cls.DISPATCHER.TOPIC.CATEGORIES.SCHEDULES,
            use_wild_card=True
        )

    @classmethod
    def on_transportation_status_message(cls, _client, user_data, topic, transportation_status_message):
        print("transportation_status_message.status.state", transportation_status_message.status.state, topic)
        if transportation_status_message.status.state == cls.DISPATCHER.TRANSPORTATION.STATE.HANDSHAKE:
            _, vehicle_config = cls.Helper.get_vehicle_config_key_and_value(
                user_data["kvs_client"], user_data["target"])
            vehicle_config["target_dispatcher"] = Topic.get_from_target(topic)

            set_flag = cls.Helper.set_vehicle_config(
                user_data["kvs_client"],
                user_data["target"],
                vehicle_config
            )
            print("vehicle_config set_flag", set_flag)
            cls.Publisher.publish_vehicle_config(user_data["mqtt_client"], user_data["target"], vehicle_config)

    @classmethod
    def on_vehicle_schedules_message(cls, _client, user_data, _topic, schedules_message):
        print("on_vehicle_schedules_message")
        cls.Helper.set_vehicle_schedules(
            user_data["kvs_client"],
            user_data["target"],
            schedules_message.schedules
        )
