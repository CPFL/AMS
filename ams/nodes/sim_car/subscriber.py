#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Target, Topic
from ams.nodes.vehicle import Subscriber as VehicleSubscriber
from ams.nodes.sim_car_dispatcher import CONST as SIM_CAR_DISPATCHER
from ams.nodes.traffic_signal import CONST as TRAFFIC_SIGNAL
from ams.nodes.sim_car import CONST, Helper, Publisher


class Subscriber(VehicleSubscriber):

    DISPATCHER = SIM_CAR_DISPATCHER
    VEHICLE = CONST

    Helper = Helper
    Publisher = Publisher

    @classmethod
    def get_vehicle_location_topic(cls):
        return Topic.get_topic(
            from_target=Target.new_target(CONST.NODE_NAME, None),
            categories=cls.VEHICLE.TOPIC.CATEGORIES.LOCATION,
            use_wild_card=True
        )

    @classmethod
    def get_traffic_signal_status_topic(cls):
        return Topic.get_topic(
            from_target=Target.new_target(TRAFFIC_SIGNAL.NODE_NAME, None),
            categories=TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        )

    @classmethod
    def on_traffic_signal_status_message(cls, _client, user_data, topic, traffic_signal_status_message):
        cls.Helper.set_traffic_signal_status(
            user_data["kvs_client"],
            user_data["target"],
            Topic.get_from_target(topic),
            traffic_signal_status_message.status
        )

    @classmethod
    def on_vehicle_location_message(cls, _client, user_data, topic, location_message):
        cls.Helper.set_vehicle_location(
            user_data["kvs_client"],
            Topic.get_from_target(topic),
            location_message.location
        )
