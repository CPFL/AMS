#!/usr/bin/env python
# coding: utf-8

import json

from ams import logger
from ams.helpers import Converter, Target
from ams.structures import TOPIC


class Topic(object):

    CONST = TOPIC
    domain = TOPIC.DOMAIN

    @staticmethod
    def get_target_topic_part(target, use_wild_card=False):
        if target is not None:
            return TOPIC.DELIMITER.join([
                Converter.camel_case_to_snake_case(target.group),
                (target.id if target.id is not None else ("+" if use_wild_card else TOPIC.ANY))
            ])
        else:
            if use_wild_card:
                return TOPIC.DELIMITER.join(["+", "+"])
            else:
                return TOPIC.DELIMITER.join([TOPIC.ANY, TOPIC.ANY])

    @staticmethod
    def get_topic(from_target, domain=None, to_target=None, categories=None, use_wild_card=False):
        if domain is None:
            domain = Topic.domain
        return TOPIC.DELIMITER.join([
            "",
            domain,
            Topic.get_target_topic_part(from_target, use_wild_card),
            Topic.get_target_topic_part(to_target, use_wild_card),
            TOPIC.DELIMITER.join(categories if categories is not None else "#")
        ])

    @staticmethod
    def get_response_topic(topic, from_target=None):
        domain = Topic.get_domain(topic)
        if from_target is None:
            from_target = Topic.get_to_target(topic)
        to_target = Topic.get_from_target(topic)
        categories = Topic.get_categories(topic)
        if categories[0] != TOPIC.REQUEST_CATEGORIES_HEAD:
            logger.error("Unknown request topic categories head: {}".format(categories[0]))
        categories[0] = TOPIC.RESPONSE_CATEGORIES_HEAD
        return Topic.get_topic(from_target, domain, to_target, categories)

    @staticmethod
    def get_domain(topic):
        return topic.split(TOPIC.DELIMITER)[TOPIC.DOMAIN_INDEX]

    @staticmethod
    def get_from_group(topic):
        return topic.split(TOPIC.DELIMITER)[TOPIC.FROM_GROUP_INDEX]

    @staticmethod
    def get_from_id(topic):
        return topic.split(TOPIC.DELIMITER)[TOPIC.FROM_ID_INDEX]

    @staticmethod
    def get_from_target(topic):
        return Target.new_target(Topic.get_from_group(topic), Topic.get_from_id(topic))

    @staticmethod
    def get_to_group(topic):
        return topic.split(TOPIC.DELIMITER)[TOPIC.TO_GROUP_INDEX]

    @staticmethod
    def get_to_id(topic):
        return topic.split(TOPIC.DELIMITER)[TOPIC.TO_ID_INDEX]

    @staticmethod
    def get_to_target(topic):
        return Target.new_target(Topic.get_to_group(topic), Topic.get_to_id(topic))

    @staticmethod
    def get_targets(topic):
        return Topic.get_from_target(topic), Topic.get_to_target(topic)

    @staticmethod
    def get_categories(topic):
        return topic.split(TOPIC.DELIMITER)[TOPIC.CATEGORIES_HEAD_INDEX:]

    @staticmethod
    def compare_topics(subscribe_topic, incoming_topic):
        if subscribe_topic == incoming_topic:
            return True
        else:
            subscribe_topic_parts = subscribe_topic.split(TOPIC.DELIMITER)
            incoming_topic_parts = incoming_topic.split(TOPIC.DELIMITER)
            for i, subscribe_topic_part in enumerate(subscribe_topic_parts):
                if subscribe_topic_part == "#":
                    return True
                else:
                    if all([
                        subscribe_topic_part != incoming_topic_parts[i],
                        subscribe_topic_part != "+",
                        TOPIC.ANY != incoming_topic_parts[i]
                    ]):
                        return False
        return True

    @staticmethod
    def serialize(message):
        return json.dumps(message)

    @staticmethod
    def unserialize(payload, structure=None):
        message = json.loads(payload)
        if structure is None:
            return message
        else:
            if isinstance(message, dict):
                return structure.new_data(**message)
            elif isinstance(message, list):
                return structure.new_data(message)
            elif message is None:
                return message
            else:
                logger.error(
                    "ValueError: message={}, type_of_message={}, payload={}, type_of_payload={}".format
                    (message, type(message), payload, type(payload)))
                raise ValueError(
                    "ValueError: message={}, type_of_message={}, payload={}, type_of_payload={}".format(
                        message, type(message), payload, type(payload)))
