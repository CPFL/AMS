#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.base import CONST, Helper


class Subscriber(object):

    CONST = CONST
    Helper = Helper

    @classmethod
    def get_request_get_config_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=None,
            to_target=target_roles["self"],
            categories=cls.CONST.TOPIC.CATEGORIES.REQUEST_GET_CONFIG,
            use_wild_card=True
        )

    @classmethod
    def get_request_get_status_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=None,
            to_target=target_roles["self"],
            categories=cls.CONST.TOPIC.CATEGORIES.REQUEST_GET_STATUS,
            use_wild_card=True
        )

    @classmethod
    def on_request_get_config_message(cls, _client, user_data, topic, request_config_message):
        response_topic = Topic.get_response_topic(topic)
        message = cls.Helper.get_response_config_message(
            user_data["clients"], user_data["target_roles"], request_config_message)
        user_data["clients"]["pubsub"].publish(response_topic, message)

    @classmethod
    def on_request_get_status_message(cls, _client, user_data, topic, request_status_message):
        response_topic = Topic.get_response_topic(topic)
        message = cls.Helper.get_response_status_message(
            user_data["clients"], user_data["target_roles"], request_status_message)
        user_data["clients"]["pubsub"].publish(response_topic, message)
