#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.infra import CONST, Helper, Publisher, StateMachine


class Subscriber(object):

    INFRA = CONST

    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine

    @classmethod
    def get_config_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["controller"] if "controller" in target_roles else None,
            to_target=target_roles[cls.INFRA.ROLE_NAME],
            categories=cls.INFRA.TOPIC.CATEGORIES.CONFIG,
            use_wild_card=True
        )

    @classmethod
    def get_schedules_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=None,
            to_target=target_roles[cls.INFRA.ROLE_NAME],
            categories=cls.INFRA.TOPIC.CATEGORIES.SCHEDULES,
            use_wild_card=True
        )

    @classmethod
    def on_config_message(cls, _client, user_data, _topic, config):
        cls.Helper.set_config(user_data["clients"], user_data["target_roles"], config)

    @classmethod
    def on_schedules_message(cls, _client, user_data, _topic, schedules):
        cls.Helper.set_schedules(user_data["clients"], user_data["target_roles"], schedules)
