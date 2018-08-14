#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.sim_autoware import CONST, Helper


class Publisher(object):

    AUTOWARE = CONST
    Helper = Helper

    @classmethod
    def get_current_pose_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["autoware"],
            to_target=target_roles["vehicle"],
            categories=cls.AUTOWARE.TOPIC.CATEGORIES.CURRENT_POSE
        )

    @classmethod
    def get_closest_waypoint_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["autoware"],
            to_target=target_roles["vehicle"],
            categories=cls.AUTOWARE.TOPIC.CATEGORIES.CLOSEST_WAYPOINT
        )

    @classmethod
    def get_decision_maker_state_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["autoware"],
            to_target=target_roles["vehicle"],
            categories=cls.AUTOWARE.TOPIC.CATEGORIES.DECISION_MAKER_STATE
        )

    @classmethod
    def publish_current_pose(cls, clients, target_roles):
        topic = cls.get_current_pose_topic(target_roles)
        message = cls.Helper.get_current_pose(clients, target_roles)
        if message is not None:
            clients["pubsub"].publish(topic, message)
            return True
        return False

    @classmethod
    def publish_closest_waypoint(cls, clients, target_roles):
        topic = cls.get_closest_waypoint_topic(target_roles)
        message = cls.Helper.get_closest_waypoint(clients, target_roles)
        if message is not None:
            clients["pubsub"].publish(topic, message)
            return True
        return False

    @classmethod
    def publish_decision_maker_state(cls, clients, target_roles):
        topic = cls.get_decision_maker_state_topic(target_roles)
        message = cls.Helper.get_decision_maker_state(clients, target_roles)
        if message is not None:
            clients["pubsub"].publish(topic, message)
            return True
        return False
