#!/usr/bin/env python
# coding: utf-8

import yaml

from ams import VERSION, AttrDict
from ams.helpers import Topic, Hook, Schedule
from ams.structures import (
    Autoware, AutowareInterface, Vehicle, Dispatcher)


class Publisher(object):

    @classmethod
    def get_current_pose_topic(cls, target_autoware, target_vehicle):
        return Topic.get_topic(
            from_target=target_autoware,
            to_target=target_vehicle,
            categories=AutowareInterface.CONST.TOPIC.CATEGORIES.CURRENT_POSE
        )

    @classmethod
    def get_closest_waypoint_topic(cls, target_autoware, target_vehicle):
        return Topic.get_topic(
            from_target=target_autoware,
            to_target=target_vehicle,
            categories=AutowareInterface.CONST.TOPIC.CATEGORIES.CLOSEST_WAYPOINT
        )

    @classmethod
    def get_decision_maker_state_topic(cls, target_autoware, target_vehicle):
        return Topic.get_topic(
            from_target=target_autoware,
            to_target=target_vehicle,
            categories=AutowareInterface.CONST.TOPIC.CATEGORIES.DECISION_MAKER_STATE
        )

    @classmethod
    def get_route_point_topic(cls, target_autoware, target_vehicle):
        return Topic.get_topic(
            from_target=target_autoware,
            to_target=target_vehicle,
            categories=AutowareInterface.CONST.TOPIC.CATEGORIES.ROUTE_POINT
        )

    @classmethod
    def get_state_cmd_topic(cls, target_vehicle, target_autoware):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_autoware,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.STATE_CMD
        )

    @classmethod
    def get_route_code_topic(cls, target_vehicle, target_autoware):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_autoware,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.ROUTE_CODE,
        )

    @classmethod
    def get_vehicle_config_topic(cls, target_vehicle, target_dispatcher):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_dispatcher,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.CONFIG
        )

    @classmethod
    def get_vehicle_status_topic(cls, target_vehicle, target_dispatcher):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_dispatcher,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.STATUS
        )

    @classmethod
    def get_geotopic_topic(cls, target_vehicle, location):
        return Topic.get_topic(
            from_target=target_vehicle,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.GEOTOPIC + list(location.geohash)
        )

    @classmethod
    def get_transportation_status_message_topic(cls, target_dispatcher, target_vehicle):
        return Topic.get_topic(
            from_target=target_dispatcher,
            to_target=target_vehicle,
            categories=Dispatcher.CONST.TOPIC.CATEGORIES.TRANSPORTATION_STATUS
        )

    @classmethod
    def get_schedules_message_topic(cls, from_target, to_target):
        return Topic.get_topic(
            from_target=from_target,
            to_target=to_target,
            categories=Dispatcher.CONST.TOPIC.CATEGORIES.SCHEDULES
        )

    @classmethod
    def publish_ros_current_pose(cls, pubsub_client, kvs_client, target_autoware, wait=False):
        message = AttrDict.get_dict(Hook.get_current_pose(kvs_client, target_autoware))
        if message is not None:
            pubsub_client.publish(Autoware.CONST.TOPIC.CURRENT_POSE, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_ros_closest_waypoint(cls, pubsub_client, kvs_client, target_autoware, wait=False):
        message = AttrDict.get_dict(Hook.get_closest_waypoint(kvs_client, target_autoware))
        if message is not None:
            pubsub_client.publish(Autoware.CONST.TOPIC.CLOSEST_WAYPOINT, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_ros_decision_maker_state(cls, pubsub_client, kvs_client, target_autoware, wait=False):
        message = AttrDict.get_dict(Hook.get_decision_maker_state(kvs_client, target_autoware))
        if message is not None:
            pubsub_client.publish(Autoware.CONST.TOPIC.DECISION_MAKER_STATE, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_current_pose(cls, pubsub_client, kvs_client, target_autoware, target_vehicle, wait=False):
        topic = cls.get_current_pose_topic(target_autoware, target_vehicle)
        message = AttrDict.get_dict(Hook.get_current_pose(kvs_client, target_autoware))
        if message is not None:
            pubsub_client.publish(topic, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_closest_waypoint(cls, pubsub_client, kvs_client, target_autoware, target_vehicle, wait=False):
        topic = cls.get_closest_waypoint_topic(target_autoware, target_vehicle)
        message = AttrDict.get_dict(Hook.get_closest_waypoint(kvs_client, target_autoware))
        if message is not None:
            pubsub_client.publish(topic, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_decision_maker_state(cls, pubsub_client, kvs_client, target_autoware, target_vehicle, wait=False):
        topic = cls.get_decision_maker_state_topic(target_autoware, target_vehicle)
        message = AttrDict.get_dict(Hook.get_decision_maker_state(kvs_client, target_autoware))
        if message is not None:
            pubsub_client.publish(topic, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_route_point(cls, pubsub_client, target_autoware, target_vehicle, route_point):
        topic = cls.get_route_point_topic(target_autoware, target_vehicle)
        message = AutowareInterface.Message.RoutePoint.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION
            },
            "body": route_point
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_vehicle_config(cls, pubsub_client, target_vehicle, target_dispatcher, vehicle_config):
        topic = cls.get_vehicle_config_topic(target_vehicle, target_dispatcher)
        message = Vehicle.Message.Config.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION
            },
            "body": vehicle_config
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_vehicle_status(cls, pubsub_client, target_vehicle, target_dispatcher, vehicle_status):
        topic = cls.get_vehicle_status_topic(target_vehicle, target_dispatcher)
        message = Vehicle.Message.Status.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION
            },
            "body": vehicle_status
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_vehicle_geotopic(cls, pubsub_client, target_vehicle, vehicle_status):
        topic = cls.get_geotopic_topic(target_vehicle, vehicle_status.location)
        pubsub_client.publish(topic, target_vehicle)

    @classmethod
    def publish_route_code(cls, pubsub_client, kvs_client, target_vehicle, target_autoware):
        schedule_id = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status).schedule_id
        if schedule_id is not None:
            schedule = Schedule.get_schedule_by_id(Hook.get_schedules(kvs_client, target_vehicle), schedule_id)
            route_code = schedule.route_code
            if route_code is not None:
                topic = cls.get_route_code_topic(target_vehicle, target_autoware)
                message = {
                    "header": {
                        "id": Schedule.get_id(),
                        "time": Schedule.get_time(),
                        "version": VERSION
                    },
                    "body": route_code
                }
                pubsub_client.publish(topic, message)

    @classmethod
    def publish_state_cmd(cls, pubsub_client, target_vehicle, target_autoware, state_cmd):
        topic = cls.get_state_cmd_topic(target_vehicle, target_autoware)
        pubsub_client.publish(topic, state_cmd)

    @classmethod
    def publish_transportation_status_message(
            cls, pubsub_client, target_dispatcher, target_vehicle, transportation_status):
        topic = cls.get_transportation_status_message_topic(target_dispatcher, target_vehicle)
        transportation_status_message = Dispatcher.Message.TransportationStatus.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION
            },
            "body": transportation_status,
        })
        pubsub_client.publish(topic, transportation_status_message)

    @classmethod
    def publish_schedules_message(cls, pubsub_client, from_target, to_target, schedules, schedules_target=None):
        if schedules_target is None:
            schedules_target = to_target
        topic = cls.get_schedules_message_topic(from_target, to_target)
        schedules_message = Dispatcher.Message.Schedules.new_data(**{
            "header": {
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION,
            },
            "body": {
                "target": schedules_target,
                "schedules": schedules
            }
        })
        pubsub_client.publish(topic, schedules_message)
