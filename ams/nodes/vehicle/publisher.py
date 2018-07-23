#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

from ams import VERSION
from ams.helpers import Topic
from ams.nodes.vehicle import CONST, Message


class Publisher(object):

    VEHICLE = CONST
    VehicleMessage = Message

    @classmethod
    def get_vehicle_config_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["vehicle"],
            to_target=target_roles["dispatcher"],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.CONFIG
        )

    @classmethod
    def get_vehicle_status_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["vehicle"],
            to_target=target_roles["dispatcher"],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.STATUS
        )

    @classmethod
    def get_geotopic_topic(cls, target_roles, location):
        return Topic.get_topic(
            from_target=target_roles["vehicle"],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.GEOTOPIC + list(location.geohash)
        )

    @classmethod
    def publish_vehicle_config(cls, clients, target_roles, vehicle_config):
        topic = cls.get_vehicle_config_topic(target_roles)
        vehicle_config_message = cls.VehicleMessage.Config.new_data(
            id=str(uuid()),
            time=time(),
            version=VERSION,
            config=vehicle_config
        )
        clients["mqtt"].publish(topic, vehicle_config_message)

    @classmethod
    def publish_vehicle_status(cls, clients, target_roles, vehicle_status):
        topic = cls.get_vehicle_status_topic(target_roles)
        vehicle_status_message = cls.VehicleMessage.Status.new_data(
            id=str(uuid()),
            time=time(),
            version=VERSION,
            status=vehicle_status
        )
        clients["mqtt"].publish(topic, vehicle_status_message)

    @classmethod
    def publish_vehicle_geotopic(cls, clients, target_roles, vehicle_status):
        topic = cls.get_geotopic_topic(target_roles["vehicle"], vehicle_status.location)
        clients["mqtt"].publish(topic, target_roles["vehicle"])
