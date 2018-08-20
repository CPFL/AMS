#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.sim_autoware import CONST, Helper


class Subscriber(object):

    AUTOWARE = CONST
    Helper = Helper

    @classmethod
    def get_lane_waypoints_array_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["vehicle"],
            to_target=target_roles["autoware"],
            categories=cls.AUTOWARE.TOPIC.CATEGORIES.LANE_WAYPOINTS_ARRAY
        )

    @classmethod
    def get_state_cmd_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["vehicle"],
            to_target=target_roles["autoware"],
            categories=cls.AUTOWARE.TOPIC.CATEGORIES.STATE_CMD
        )

    @classmethod
    def get_light_color_topic(cls, target_roles):
        return Topic.get_topic(
            from_target=target_roles["vehicle"],
            to_target=target_roles["autoware"],
            categories=cls.AUTOWARE.TOPIC.CATEGORIES.LIGHT_COLOR
        )

    @classmethod
    def on_lane_waypoints_array(cls, _client, user_data, _topic, lane_waypoints_array):
        cls.Helper.set_lane_waypoints_array(
            user_data["clients"], user_data["target_roles"], lane_waypoints_array
        )

    @classmethod
    def on_state_cmd(cls, _client, user_data, _topic, state_cmd):
        cls.Helper.set_state_cmd(
            user_data["clients"], user_data["target_roles"], state_cmd
        )

    @classmethod
    def on_light_color(cls, _client, user_data, _topic, light_color):
        cls.Helper.set_light_color(
            user_data["clients"], user_data["target_roles"], light_color
        )
