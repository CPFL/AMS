#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Target, Topic
from ams.nodes.vehicle import Subscriber as VehicleSubscriber
from ams.nodes.autoware_dispatcher import CONST as AUTOWARE_DISPATCHER
from ams.nodes.traffic_signal import CONST as TRAFFIC_SIGNAL
from ams.nodes.autoware import CONST, Helper, Publisher, StateMachine


class Subscriber(VehicleSubscriber):

    DISPATCHER = AUTOWARE_DISPATCHER
    VEHICLE = CONST

    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine

    @classmethod
    def get_current_pose_topic(cls, target_ros, target_vehicle):
        return Topic.get_topic(
            from_target=target_ros,
            to_target=target_vehicle,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.CURRENT_POSE,
        )

    @classmethod
    def get_closest_waypoint_topic(cls, target_ros, target_vehicle):
        return Topic.get_topic(
            from_target=target_ros,
            to_target=target_vehicle,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.CLOSEST_WAYPOINT,
        )

    @classmethod
    def get_decision_maker_state_topic(cls, target_ros, target_vehicle):
        return Topic.get_topic(
            from_target=target_ros,
            to_target=target_vehicle,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.DECISION_MAKER_STATE,
        )

    @classmethod
    def get_traffic_signal_status_topic(cls):
        return Topic.get_topic(
            from_target=Target.new_target(TRAFFIC_SIGNAL.NODE_NAME, None),
            categories=TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS,
        )

    @classmethod
    def on_current_pose_ros_message(cls, _client, user_data, topic, current_pose):
        _, vehicle_status = cls.Helper.get_vehicle_status_key_and_value(
            user_data["kvs_client"],
            user_data["target"]
        )

        vehicle_status.current_pose = current_pose

        cls.Helper.set_vehicle_status(
            user_data["kvs_client"],
            user_data["target"],
            vehicle_status
        )

    @classmethod
    def on_closest_waypoint_ros_message(cls, _client, user_data, topic, closest_waypoint):
        _, vehicle_status = cls.Helper.get_vehicle_status_key_and_value(
            user_data["kvs_client"],
            user_data["target"]
        )

        vehicle_status.closest_waypoint = closest_waypoint

        cls.Helper.set_vehicle_status(
            user_data["kvs_client"],
            user_data["target"],
            vehicle_status
        )

    @classmethod
    def on_decision_maker_state_ros_message(cls, _client, user_data, topic, decision_maker_state):
        _, vehicle_status = cls.Helper.get_vehicle_status_key_and_value(
            user_data["kvs_client"],
            user_data["target"]
        )

        print("Autoware.Subscriber.on_decision_maker_state_ros_message", vehicle_status)

        vehicle_status.decision_maker_state = decision_maker_state

        set_flag = cls.Helper.set_vehicle_status(
            user_data["kvs_client"],
            user_data["target"],
            vehicle_status
        )

        if set_flag:
            _, vehicle_config = cls.Helper.get_vehicle_config_key_and_value(
                user_data["kvs_client"],
                user_data["target"]
            )
            cls.StateMachine.update_vehicle_state(
                user_data["target"],
                user_data["kvs_client"],
                user_data["mqtt_client"],
                user_data["maps_client"],
                vehicle_config["target_ros"]
            )

        cls.Publisher.publish_vehicle_status(
            user_data["mqtt_client"],
            user_data["target"],
            vehicle_status
        )

    @classmethod
    def on_traffic_signal_status_message(cls, _client, user_data, topic, traffic_signal_status_message):

        # filter traffic_signal_status

        cls.Helper.set_traffic_signal_status(
            user_data["kvs_client"],
            user_data["target"],
            Topic.get_from_target(topic),
            traffic_signal_status_message.status
        )
