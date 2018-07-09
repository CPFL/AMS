#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

from ams.helpers import Target, Topic
from ams.nodes.vehicle import Publisher as VehiclePublisher
from ams.nodes.sim_car import CONST, Message


class Publisher(VehiclePublisher):

    VEHICLE = CONST
    VehicleMessage = Message

    @classmethod
    def get_vehicle_location_message_topic(cls, target):
        return Topic.get_topic(
            from_target=Target.new_target(CONST.NODE_NAME, target.id),
            categories=cls.VEHICLE.TOPIC.CATEGORIES.LOCATION
        )

    @classmethod
    def publish_vehicle_location_message(cls, mqtt_client, target, vehicle_location):
        topic = cls.get_vehicle_location_message_topic(target)
        vehicle_location_message = cls.VehicleMessage.Location.new_data(
            id=str(uuid()),
            time=time(),
            location=vehicle_location
        )
        mqtt_client.publish(topic, vehicle_location_message)
