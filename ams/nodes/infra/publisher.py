#!/usr/bin/env python
# coding: utf-8

from ams import VERSION
from ams.helpers import Topic
from ams.nodes.infra import CONST, Message, Helper


class Publisher(object):

    CONST = CONST
    Message = Message
    Helper = Helper

    @classmethod
    def get_config_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.CONST.ROLE_NAME],
            to_target=target_roles["controller"],
            categories=cls.CONST.TOPIC.CATEGORIES.CONFIG
        )

    @classmethod
    def get_status_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.CONST.ROLE_NAME],
            to_target=target_roles["controller"],
            categories=cls.CONST.TOPIC.CATEGORIES.STATUS
        )

    @classmethod
    def publish_config(cls, clients, target_roles, config):
        topic = cls.get_config_topic(target_roles)
        config_message = cls.Message.Config.new_data(
            header=cls.Message.Header.new_data(
                id=cls.Helper.get_id(),
                time=cls.Helper.get_time(),
                version=VERSION
            ),
            body=config
        )
        clients["pubsub"].publish(topic, config_message)

    @classmethod
    def publish_status(cls, clients, target_roles, status):
        topic = cls.get_status_topic(target_roles)
        status_message = cls.Message.Status.new_data(
            header=cls.Message.Header.new_data(
                id=cls.Helper.get_id(),
                time=cls.Helper.get_time(),
                version=VERSION
            ),
            body=status
        )
        clients["pubsub"].publish(topic, status_message)
