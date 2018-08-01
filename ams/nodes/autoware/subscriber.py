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
    def get_current_pose_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROS.ROLE_NAME],
            to_target=target_roles[cls.VEHICLE.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.CURRENT_POSE,
        )

    @classmethod
    def get_closest_waypoint_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROS.ROLE_NAME],
            to_target=target_roles[cls.VEHICLE.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.CLOSEST_WAYPOINT,
        )

    @classmethod
    def get_decision_maker_state_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROS.ROLE_NAME],
            to_target=target_roles[cls.VEHICLE.ROLE_NAME],
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
        cls.Helper.set_current_pose(user_data["clients"], user_data["target_roles"], current_pose)

    @classmethod
    def on_closest_waypoint_ros_message(cls, _client, user_data, topic, closest_waypoint):
        cls.Helper.set_closest_waypoint(user_data["clients"], user_data["target_roles"], closest_waypoint)

    @classmethod
    def on_decision_maker_state_ros_message(cls, _client, user_data, topic, decision_maker_state):
        _, vehicle_status = cls.Helper.get_vehicle_status_key_and_value(
            user_data["clients"],
            user_data["target_roles"]
        )

        if vehicle_status is not None:
            vehicle_status.current_pose = cls.Helper.get_current_pose(user_data["clients"], user_data["target_roles"])
            vehicle_status.closest_waypoint = cls.Helper.get_closest_waypoint(user_data["clients"],
                                                                              user_data["target_roles"])
            vehicle_status.decision_maker_state = decision_maker_state

            set_flag = cls.Helper.set_vehicle_status(
                user_data["clients"],
                user_data["target_roles"],
                vehicle_status
            )

            if set_flag:
                _, vehicle_config = cls.Helper.get_vehicle_config_key_and_value(
                    user_data["clients"],
                    user_data["target_roles"]
                )
                cls.StateMachine.update_vehicle_state(
                    user_data["clients"],
                    user_data["target_roles"]
                )

            cls.Publisher.publish_vehicle_status(
                user_data["clients"],
                user_data["target_roles"],
                vehicle_status
            )

    @classmethod
    def on_traffic_signal_status_message(cls, _client, user_data, topic, traffic_signal_status_message):

        # filter traffic_signal_status

        cls.Helper.set_traffic_signal_status(
            user_data["clients"],
            user_data["target_roles"],
            Topic.get_from_target(topic),
            traffic_signal_status_message.status
        )
