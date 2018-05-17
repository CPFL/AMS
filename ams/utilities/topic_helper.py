#!/usr/bin/env python
# coding: utf-8

import json
from ams import Converter, Target
from ams.structures import TOPIC_HELPER


class TopicHelper(object):

    CONST = TOPIC_HELPER

    @staticmethod
    def get_target_topic_part(target, use_wild_card=False):
        if target is not None:
            return TOPIC_HELPER.DELIMITER.join([
                Converter.camel_case_to_snake_case(target.group),
                (target.id if target.id is not None else ("+" if use_wild_card else TOPIC_HELPER.ANY))
            ])
        else:
            if use_wild_card:
                return TOPIC_HELPER.DELIMITER.join(["+", "+"])
            else:
                return TOPIC_HELPER.DELIMITER.join([TOPIC_HELPER.ANY, TOPIC_HELPER.ANY])

    @staticmethod
    def get_topic(from_target, domain=TOPIC_HELPER.DOMAIN, to_target=None, categories=None, use_wild_card=False):
        return TOPIC_HELPER.DELIMITER.join([
            "",
            domain,
            TopicHelper.get_target_topic_part(from_target, use_wild_card),
            TopicHelper.get_target_topic_part(to_target, use_wild_card),
            TOPIC_HELPER.DELIMITER.join(categories if categories is not None else "#")
        ])

    @staticmethod
    def get_respose_topic(topic, from_target=None):
        domain = TopicHelper.get_domain(topic)
        if from_target is None:
            from_target = TopicHelper.get_to_target(topic)
        to_target = TopicHelper.get_from_target(topic)
        categories = TopicHelper.get_categories(topic)
        return TopicHelper.get_topic(from_target, domain, to_target, categories)

    @staticmethod
    def get_domain(topic):
        return topic.split(TOPIC_HELPER.DELIMITER)[TOPIC_HELPER.DOMAIN_INDEX]

    @staticmethod
    def get_from_group(topic):
        return topic.split(TOPIC_HELPER.DELIMITER)[TOPIC_HELPER.FROM_GROUP_INDEX]

    @staticmethod
    def get_from_id(topic):
        return topic.split(TOPIC_HELPER.DELIMITER)[TOPIC_HELPER.FROM_ID_INDEX]

    @staticmethod
    def get_from_target(topic):
        return Target.new_data(
            group=TopicHelper.get_from_group(topic),
            id=TopicHelper.get_from_id(topic)
        )

    @staticmethod
    def get_to_group(topic):
        return topic.split(TOPIC_HELPER.DELIMITER)[TOPIC_HELPER.TO_GROUP_INDEX]

    @staticmethod
    def get_to_id(topic):
        return topic.split(TOPIC_HELPER.DELIMITER)[TOPIC_HELPER.TO_ID_INDEX]

    @staticmethod
    def get_to_target(topic):
        return Target.new_data(
            group=TopicHelper.get_to_group(topic),
            id=TopicHelper.get_to_id(topic)
        )

    @staticmethod
    def get_targets(topic):
        return TopicHelper.get_from_target(topic), TopicHelper.get_to_target(topic)

    @staticmethod
    def get_categories(topic):
        return topic.split(TOPIC_HELPER.DELIMITER)[TOPIC_HELPER.CATEGORIES_HEAD_INDEX]

    @staticmethod
    def compare_topics(subscribe_topic, incoming_topic):
        if subscribe_topic == incoming_topic:
            return True
        elif len(subscribe_topic) < len(incoming_topic):
            subscribe_topic_parts = subscribe_topic.split(TOPIC_HELPER.DELIMITER)
            incoming_topic_parts = incoming_topic.split(TOPIC_HELPER.DELIMITER)
            for i, subscribe_topic_part in enumerate(subscribe_topic_parts):
                if subscribe_topic_parts == "#":
                    return True
                else:
                    if all([
                        subscribe_topic_part != incoming_topic_parts[i],
                        subscribe_topic_part != "+",
                        TOPIC_HELPER.ANY != incoming_topic_parts[i]
                    ]):
                        break
        return False
