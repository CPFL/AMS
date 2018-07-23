#!/usr/bin/env python
# coding: utf-8

from ams import VERSION
from ams.helpers import Topic
from ams.nodes.dispatcher import CONST, Message, Helper


class Publisher(object):

    DISPATCHER = CONST
    Message = Message
    Helper = Helper

    @classmethod
    def get_transportation_status_message_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["dispatcher"],
            to_target=target_roles["vehicle"],
            categories=cls.DISPATCHER.TOPIC.CATEGORIES.TRANSPORTATION_STATUS
        )

    @classmethod
    def get_schedules_message_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["dispatcher"],
            to_target=target_roles["vehicle"],
            categories=cls.DISPATCHER.TOPIC.CATEGORIES.SCHEDULES
        )

    @classmethod
    def publish_transportation_status_message(
            cls, clients, target_roles, transportation_status):
        topic = cls.get_transportation_status_message_topic(target_roles)
        transportation_status_message = cls.Message.TransportationStatus.new_data(
            id=cls.Helper.get_uuid(),
            time=cls.Helper.get_current_time(),
            version=VERSION,
            status=transportation_status,
        )
        clients["mqtt"].publish(topic, transportation_status_message)

    @classmethod
    def publish_schedules_message(cls, clients, target_roles, schedules):
        topic = cls.get_schedules_message_topic(target_roles)
        schedules_message = cls.Message.Schedules.new_data(
            id=cls.Helper.get_uuid(),
            time=cls.Helper.get_current_time(),
            version=VERSION,
            schedules=schedules
        )
        clients["mqtt"].publish(topic, schedules_message)
