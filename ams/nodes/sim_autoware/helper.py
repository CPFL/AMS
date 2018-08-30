#!/usr/bin/env python
# coding: utf-8

from time import time
from math import modf

from ams.structures import CLIENT
from ams.helpers import Target, Schedule
from ams.nodes.base import Helper as BaseHelper
from ams.nodes.sim_autoware import CONST, Structure, Message


class Helper(BaseHelper):

    CONST = CONST
    Structure = Structure
    Message = Message

    @staticmethod
    def get_timestamp():
        nsec, sec = modf(time())
        return Structure.Status.CurrentPose.Header.Timestamp.new_data(
            secs=int(sec),
            nsecs=int(nsec*(10**9))
        )

    @classmethod
    def get_closest_waypoint_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles["autoware"]),
            ] + cls.CONST.TOPIC.CATEGORIES.CLOSEST_WAYPOINT)

    @classmethod
    def get_current_pose_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles["autoware"]),
            ] + cls.CONST.TOPIC.CATEGORIES.CURRENT_POSE)

    @classmethod
    def get_state_cmd_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles["autoware"]),
            ] + cls.CONST.TOPIC.CATEGORIES.STATE_CMD)

    @classmethod
    def get_lane_waypoints_array_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles["autoware"]),
            ] + cls.CONST.TOPIC.CATEGORIES.LANE_WAYPOINTS_ARRAY)

    @classmethod
    def get_decision_maker_state_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles["autoware"]),
            ] + cls.CONST.TOPIC.CATEGORIES.DECISION_MAKER_STATE)

    @classmethod
    def get_light_color_key(cls, target_roles):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles["autoware"]),
            ] + cls.CONST.TOPIC.CATEGORIES.LIGHT_COLOR)

    @classmethod
    def get_closest_waypoint(cls, clients, target_roles):
        key = cls.get_closest_waypoint_key(target_roles)
        return clients["kvs"].get(key)

    @classmethod
    def get_current_pose(cls, clients, target_roles):
        key = cls.get_current_pose_key(target_roles)
        return clients["kvs"].get(key)

    @classmethod
    def get_state_cmd(cls, clients, target_roles):
        key = cls.get_state_cmd_key(target_roles)
        return clients["kvs"].get(key)

    @classmethod
    def get_lane_waypoints_array(cls, clients, target_roles):
        key = cls.get_lane_waypoints_array_key(target_roles)
        return clients["kvs"].get(key)

    @classmethod
    def get_decision_maker_state(cls, clients, target_roles):
        key = cls.get_decision_maker_state_key(target_roles)
        return clients["kvs"].get(key)

    @classmethod
    def get_light_color(cls, clients, target_roles):
        key = cls.get_light_color_key(target_roles)
        return clients["kvs"].get(key)

    @classmethod
    def get_status(cls, clients, target_roles):
        return cls.Structure.Status.new_data(
            current_pose=cls.get_current_pose(clients, target_roles),
            closest_waypoint=cls.get_closest_waypoint(clients, target_roles),
            state_cmd=cls.get_state_cmd(clients, target_roles),
            lane_waypoints_array=cls.get_lane_waypoints_array(clients, target_roles),
            decision_maker_state=cls.get_decision_maker_state(clients, target_roles),
            light_color=cls.get_light_color(clients, target_roles),
            updated_at=Schedule.get_time()
        )

    @classmethod
    def get_lane_array(cls, clients, route_code):
        return clients["maps"].route.get_lane_array(route_code)

    @classmethod
    def set_status(cls, clients, target_roles, value):
        set_flag = cls.set_current_pose(clients, target_roles, value.current_pose)
        set_flag *= cls.set_closest_waypoint(clients, target_roles, value.closest_waypoint)
        set_flag *= cls.set_state_cmd(clients, target_roles, value.state_cmd)
        set_flag *= cls.set_lane_waypoints_array(clients, target_roles, value.lane_waypoints_array)
        set_flag *= cls.set_decision_maker_state(clients, target_roles, value.decision_maker_state)
        return set_flag

    @classmethod
    def set_closest_waypoint(cls, clients, target_roles, value):
        key = cls.get_closest_waypoint_key(target_roles)
        return clients["kvs"].set(key, value)

    @classmethod
    def set_current_pose(cls, clients, target_roles, value):
        key = cls.get_current_pose_key(target_roles)
        return clients["kvs"].set(key, value)

    @classmethod
    def set_state_cmd(cls, clients, target_roles, value):
        key = cls.get_state_cmd_key(target_roles)
        return clients["kvs"].set(key, value)

    @classmethod
    def set_lane_waypoints_array(cls, clients, target_roles, value):
        key = cls.get_lane_waypoints_array_key(target_roles)
        return clients["kvs"].set(key, value)

    @classmethod
    def set_decision_maker_state(cls, clients, target_roles, value):
        key = cls.get_decision_maker_state_key(target_roles)
        return clients["kvs"].set(key, value)

    @classmethod
    def initialize_closest_waypoint(cls, clients, target_roles):
        return cls.set_closest_waypoint(clients, target_roles, cls.Structure.Status.ClosestWaypoint.new_data(data=0))

    @classmethod
    def initialize_state_cmd(cls, clients, target_roles):
        return cls.set_state_cmd(clients, target_roles, None)

    @classmethod
    def initialize_lane_waypoints_array(cls, clients, target_roles):
        return cls.set_lane_waypoints_array(clients, target_roles, None)

    @classmethod
    def update_closest_waypoint(cls, clients, target_roles, config, status):
        status.closest_waypoint.data = min(
            status.closest_waypoint.data + config.step_size, len(status.lane_waypoints_array.lanes[0].waypoints) - 1)
        return cls.set_closest_waypoint(clients, target_roles, status.closest_waypoint)

    @classmethod
    def update_current_pose(cls, clients, target_roles, status):
        if status.lane_waypoints_array is not None:
            if 0 <= status.closest_waypoint.data < len(status.lane_waypoints_array.lanes[0].waypoints):
                return cls.set_current_pose(
                    clients, target_roles,
                    status.lane_waypoints_array.lanes[0].waypoints[status.closest_waypoint.data].pose)
        return False
