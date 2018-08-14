#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.vehicle import CONST as VEHICLE
from ams.nodes.dispatcher import Helper, StateMachine


class Subscriber(object):

    Helper = Helper
    StateMachine = StateMachine

    VEHICLE = VEHICLE

    @classmethod
    def get_vehicle_config_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.CONFIG,
            use_wild_card=True
        )

    @classmethod
    def get_vehicle_status_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        )

    @classmethod
    def on_user_status_message(cls, _client, user_data, topic, user_status_message):
        user_data["target_roles"]["user"] = Topic.get_from_target(topic)
        set_flag = cls.Helper.set_user_status(
            user_data["clients"], user_data["target_roles"], user_status_message.status)

        if set_flag:
            cls.StateMachine.update_transportation_state(user_data["clients"], user_data["target_roles"])

    @classmethod
    def on_vehicle_config_message(cls, _client, user_data, topic, vehicle_config_message):
        user_data["target_roles"][cls.VEHICLE.ROLE_NAME] = Topic.get_from_target(topic)
        cls.Helper.set_vehicle_config(user_data["clients"], user_data["target_roles"], vehicle_config_message.config)

    @classmethod
    def on_vehicle_status_message(cls, _client, user_data, topic, vehicle_status_message):
        user_data["target_roles"][cls.VEHICLE.ROLE_NAME] = Topic.get_from_target(topic)
        set_flag = cls.Helper.set_vehicle_status(
            user_data["clients"], user_data["target_roles"], vehicle_status_message.status)

        if set_flag:
            cls.StateMachine.update_transportation_state(user_data["clients"], user_data["target_roles"])
