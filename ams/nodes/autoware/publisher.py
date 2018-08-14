#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.vehicle import Publisher as VehiclePublisher
from ams.nodes.autoware import CONST


class Publisher(VehiclePublisher):

    VEHICLE = CONST

    @classmethod
    def get_lane_waypoint_array_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROLE_NAME],
            to_target=target_roles[cls.VEHICLE.ROS.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.BASED_LANE_WAYPOINTS_ARRAY,
        )

    @classmethod
    def get_state_cmd_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles[cls.VEHICLE.ROLE_NAME],
            to_target=target_roles[cls.VEHICLE.ROS.ROLE_NAME],
            categories=cls.VEHICLE.TOPIC.CATEGORIES.STATE_CMD,
        )

    @classmethod
    def publish_lane_waypoint_array(cls, clients, target_roles, lane_waypoint_array):
        topic = cls.get_lane_waypoint_array_topic(target_roles)
        clients["pubsub"].publish(topic, lane_waypoint_array)

    @classmethod
    def publish_state_cmd(cls, clients, target_roles, state_cmd):
        topic = cls.get_state_cmd_topic(target_roles)
        clients["pubsub"].publish(topic, state_cmd)
