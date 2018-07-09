#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

from ams.helpers import Topic
from ams.nodes.dispatcher import CONST, Message


class Publisher(object):

    DISPATCHER = CONST

    @classmethod
    def get_transportation_status_message_topic(cls, target_dispatcher, target_vehicle):
        return Topic.get_topic(
            from_target=target_dispatcher,
            to_target=target_vehicle,
            categories=cls.DISPATCHER.TOPIC.CATEGORIES.TRANSPORTATION_STATUS
        )

    @classmethod
    def get_schedules_message_topic(cls, target_dispatcher, target_vehicle):
        return Topic.get_topic(
            from_target=target_dispatcher,
            to_target=target_vehicle,
            categories=cls.DISPATCHER.TOPIC.CATEGORIES.SCHEDULES
        )

    @classmethod
    def publish_transportation_status_message(
            cls, mqtt_client, target_dispatcher, target_vehicle, transportation_status):
        topic = cls.get_transportation_status_message_topic(target_dispatcher, target_vehicle)
        transportation_status_message = Message.TransportationStatus.new_data(
            id=str(uuid()),
            time=time(),
            status=transportation_status
        )
        mqtt_client.publish(topic, transportation_status_message)

    @classmethod
    def publish_schedules_message(cls, mqtt_client, target_dispatcher, target_vehicle, schedules):
        topic = cls.get_schedules_message_topic(target_dispatcher, target_vehicle)
        schedules_message = Message.Schedules.new_data(
            id=str(uuid()),
            time=time(),
            schedules=schedules
        )
        mqtt_client.publish(topic, schedules_message)
