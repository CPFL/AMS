#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

from ams.helpers import Topic
from ams.nodes.vehicle import CONST, Message


class Publisher(object):

    VEHICLE = CONST
    VehicleMessage = Message

    @classmethod
    def get_vehicle_config_topic(cls, target_vehicle, target_dispatcher):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_dispatcher,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.CONFIG
        )

    @classmethod
    def get_vehicle_status_topic(cls, target_vehicle, target_dispatcher):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_dispatcher,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.STATUS
        )

    @classmethod
    def get_geotopic_topic(cls, target_vehicle, location):
        return Topic.get_topic(
            from_target=target_vehicle,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.GEOTOPIC + list(location.geohash)
        )

    @classmethod
    def publish_vehicle_config(cls, mqtt_client, target_vehicle, vehicle_config):
        topic = cls.get_vehicle_config_topic(target_vehicle, vehicle_config.target_dispatcher)
        print("publish_vehicle_config topic", topic, target_vehicle, vehicle_config)
        vehicle_config_message = cls.VehicleMessage.Config.new_data(
            id=str(uuid()),
            time=time(),
            config=vehicle_config
        )
        mqtt_client.publish(topic, vehicle_config_message)

    @classmethod
    def publish_vehicle_status(cls, mqtt_client, target_vehicle, vehicle_status, target_dispatcher):
        topic = cls.get_vehicle_status_topic(target_vehicle, target_dispatcher)
        vehicle_status_message = cls.VehicleMessage.Status.new_data(
            id=str(uuid()),
            time=time(),
            status=vehicle_status
        )
        mqtt_client.publish(topic, vehicle_status_message)

    @classmethod
    def publish_vehicle_geotopic(cls, mqtt_client, target_vehicle, vehicle_status):
        topic = cls.get_geotopic_topic(target_vehicle, vehicle_status.location)
        mqtt_client.publish(topic, target_vehicle)
