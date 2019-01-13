#!/usr/bin/env python
# coding: utf-8

import yaml

from ams import VERSION, AttrDict
from ams.helpers import Topic, Hook, Event, Target
from ams.structures import (
    Autoware, AutowareInterface, Vehicle, Dispatcher, TrafficSignal, User)


class Publisher(object):

    @classmethod
    def get_current_pose_rostopic(cls, target_autoware=None):
        rostopic = Autoware.CONST.TOPIC.CURRENT_POSE
        if target_autoware is not None:
            return Topic.CONST.DELIMITER.join([
                rostopic,
                Target.encode(target_autoware)
            ])
        return rostopic

    @classmethod
    def get_vehicle_location_rostopic(cls, target_autoware=None):
        rostopic = Autoware.CONST.TOPIC.VEHICLE_LOCATION
        if target_autoware is not None:
            return Topic.CONST.DELIMITER.join([
                rostopic,
                Target.encode(target_autoware)
            ])
        return rostopic

    @classmethod
    def get_decision_maker_state_rostopic(cls, target_autoware=None):
        rostopic = Autoware.CONST.TOPIC.DECISION_MAKER_STATE
        if target_autoware is not None:
            return Topic.CONST.DELIMITER.join([
                rostopic,
                Target.encode(target_autoware)
            ])
        return rostopic

    @classmethod
    def get_lane_array_rostopic(cls, target_autoware=None):
        rostopic = AutowareInterface.CONST.TOPIC.LANE_ARRAY
        if target_autoware is not None:
            return Topic.CONST.DELIMITER.join([
                rostopic,
                Target.encode(target_autoware)
            ])
        return rostopic

    @classmethod
    def get_state_cmd_rostopic(cls, target_autoware=None):
        rostopic = AutowareInterface.CONST.TOPIC.STATE_CMD
        if target_autoware is not None:
            return Topic.CONST.DELIMITER.join([
                rostopic,
                Target.encode(target_autoware)
            ])
        return rostopic

    @classmethod
    def get_stop_waypoint_index_rostopic(cls, target_autoware=None):
        rostopic = AutowareInterface.CONST.TOPIC.STOP_WAYPOINT_INDEX
        if target_autoware is not None:
            return Topic.CONST.DELIMITER.join([
                rostopic,
                Target.encode(target_autoware)
            ])
        return rostopic

    @classmethod
    def get_current_pose_topic(cls, target_autoware, target_vehicle):
        return Topic.get_topic(
            from_target=target_autoware,
            to_target=target_vehicle,
            categories=AutowareInterface.CONST.TOPIC.CATEGORIES.CURRENT_POSE
        )

    @classmethod
    def get_vehicle_location_topic(cls, target_autoware, target_vehicle):
        return Topic.get_topic(
            from_target=target_autoware,
            to_target=target_vehicle,
            categories=AutowareInterface.CONST.TOPIC.CATEGORIES.VEHICLE_LOCATION
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
    def get_decision_maker_state_message_topic(cls, target_autoware, target_vehicle):
        return Topic.get_topic(
            from_target=target_autoware,
            to_target=target_vehicle,
            categories=AutowareInterface.CONST.TOPIC.CATEGORIES.DECISION_MAKER_STATE
        )

    @classmethod
    def get_state_cmd_topic(cls, target_vehicle, target_autoware):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_autoware,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.STATE_CMD
        )

    @classmethod
    def get_stop_route_point_topic(cls, target_vehicle, target_autoware):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_autoware,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.STOP_ROUTE_POINT
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
    def get_config_topic(cls, from_target, to_target):
        return Topic.get_topic(
            from_target=from_target,
            to_target=to_target,
            categories=["config"]
        )

    @classmethod
    def get_status_topic(cls, from_target, to_target):
        return Topic.get_topic(
            from_target=from_target,
            to_target=to_target,
            categories=["status"]
        )

    @classmethod
    def get_geotopic_topic(cls, target_vehicle, location):
        return Topic.get_topic(
            from_target=target_vehicle,
            categories=Vehicle.CONST.TOPIC.CATEGORIES.GEOTOPIC + list(location.geohash)
        )

    @classmethod
    def get_schedule_message_topic(cls, from_target, to_target):
        return Topic.get_topic(
            from_target=from_target,
            to_target=to_target,
            categories=Dispatcher.CONST.TOPIC.CATEGORIES.SCHEDULE
        )

    @classmethod
    def get_event_message_topic(cls, from_target, to_target):
        return Topic.get_topic(
            from_target=from_target,
            to_target=to_target,
            categories=Dispatcher.CONST.TOPIC.CATEGORIES.EVENT
        )

    @classmethod
    def get_vehicle_info_message_topic(cls, from_target, to_target):
        return Topic.get_topic(
            from_target=from_target,
            to_target=to_target,
            categories=Dispatcher.CONST.TOPIC.CATEGORIES.VEHICLE_INFO
        )

    @classmethod
    def publish_ros_current_pose(cls, pubsub_client, kvs_client, target_autoware, wait=False, identifiable=False):
        message = AttrDict.get_dict(Hook.get_current_pose(kvs_client, target_autoware))
        if message is not None:
            if identifiable:
                rostopic = cls.get_current_pose_rostopic(target_autoware)
            else:
                rostopic = cls.get_current_pose_rostopic()
            pubsub_client.publish(rostopic, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_ros_vehicle_location(cls, pubsub_client, kvs_client, target_autoware, wait=False, identifiable=False):
        message = AttrDict.get_dict(Hook.get_vehicle_location(kvs_client, target_autoware))
        if message is not None:
            if identifiable:
                rostopic = cls.get_vehicle_location_rostopic(target_autoware)
            else:
                rostopic = cls.get_vehicle_location_rostopic()
            pubsub_client.publish(rostopic, yaml.dump(message), wait=wait)
            return True
        return False

    @classmethod
    def publish_ros_decision_maker_state(
            cls, pubsub_client, kvs_client, target_autoware, wait=False, identifiable=False):
        message = AttrDict.get_dict(Hook.get_decision_maker_state(kvs_client, target_autoware))
        if message is not None:
            if identifiable:
                rostopic = cls.get_decision_maker_state_rostopic(target_autoware)
            else:
                rostopic = cls.get_decision_maker_state_rostopic()
            pubsub_client.publish(rostopic, yaml.dump(message), wait=wait)
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
    def publish_vehicle_location(cls, pubsub_client, kvs_client, target_autoware, target_vehicle, wait=False):
        topic = cls.get_vehicle_location_topic(target_autoware, target_vehicle)
        message = AttrDict.get_dict(Hook.get_vehicle_location(kvs_client, target_autoware))
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
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION
            },
            "body": route_point
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_decision_maker_state_message(
            cls, pubsub_client, kvs_client, target_autoware_interface, target_autoware, target_vehicle):
        decision_maker_state = Hook.get_decision_maker_state(kvs_client, target_autoware_interface)
        topic = cls.get_decision_maker_state_message_topic(target_autoware, target_vehicle)
        message = AutowareInterface.Message.DecisionMakerState.new_data(**{
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION,
            },
            "body": decision_maker_state
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_config(cls, pubsub_client, kvs_client, from_target, to_target, structure):
        config = Hook.get_config(kvs_client, from_target, structure.Config)
        if config is None:
            return
        topic = cls.get_config_topic(from_target, to_target)
        message = Vehicle.Message.Config.new_data(**{
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION
            },
            "body": config
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_status(cls, pubsub_client, kvs_client, from_target, to_target, structure):
        status = Hook.get_status(kvs_client, from_target, structure.Status)
        if status is None:
            return
        topic = cls.get_status_topic(from_target, to_target)
        message = structure.Message.Status.new_data(**{
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION
            },
            "body": status
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_vehicle_status(cls, pubsub_client, kvs_client, target_vehicle, target_dispatcher):
        cls.publish_status(pubsub_client, kvs_client, target_vehicle, target_dispatcher, Vehicle)

    @classmethod
    def publish_traffic_signal_status(cls, pubsub_client, kvs_client, from_target, to_target):
        cls.publish_status(pubsub_client, kvs_client, from_target, to_target, TrafficSignal)

    @classmethod
    def publish_user_status(cls, pubsub_client, kvs_client, target_user, target_dispatcher):
        cls.publish_status(pubsub_client, kvs_client, target_user, target_dispatcher, User)

    @classmethod
    def publish_dispatcher_status(cls, pubsub_client, kvs_client, target_dispatcher, target_vehicle, to_target):
        cls.publish_status(
            pubsub_client, kvs_client, target_dispatcher, to_target, Dispatcher, sub_target=target_vehicle)

    @classmethod
    def publish_vehicle_geotopic(cls, pubsub_client, target_vehicle, vehicle_status):
        topic = cls.get_geotopic_topic(target_vehicle, vehicle_status.location)
        pubsub_client.publish(topic, target_vehicle)

    @classmethod
    def publish_route_code(cls, pubsub_client, kvs_client, target_vehicle, target_autoware):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is None:
            return
        event_id = vehicle_status.event_id
        if event_id is not None:
            event = Event.get_event_by_id(Hook.get_schedule(kvs_client, target_vehicle).events, event_id)
            if event.name == Vehicle.CONST.EVENT.SEND_LANE_ARRAY:
                if event.route_code is not None:
                    topic = cls.get_route_code_topic(target_vehicle, target_autoware)
                    message = {
                        "header": {
                            "id": Event.get_id(),
                            "time": Event.get_time(),
                            "version": VERSION
                        },
                        "body": event.route_code
                    }
                    pubsub_client.publish(topic, message)

    @classmethod
    def publish_state_cmd(cls, pubsub_client, target_vehicle, target_autoware, state_cmd):
        topic = cls.get_state_cmd_topic(target_vehicle, target_autoware)
        pubsub_client.publish(topic, state_cmd)

    @classmethod
    def publish_stop_route_point(cls, pubsub_client, kvs_client, maps_client, target_vehicle, target_autoware):
        topic = cls.get_stop_route_point_topic(target_vehicle, target_autoware)
        message = {
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION
            },
            "body": Hook.generate_vehicle_stop_route_point(kvs_client, maps_client, target_vehicle)
        }
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_schedule_message(cls, pubsub_client, from_target, to_target, schedule, schedule_target=None):
        if schedule_target is None:
            schedule_target = to_target
        topic = cls.get_schedule_message_topic(from_target, to_target)
        schedule_message = Dispatcher.Message.Schedule.new_data(**{
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION,
            },
            "body": {
                "target": schedule_target,
                "schedule": schedule
            }
        })
        pubsub_client.publish(topic, schedule_message)

    @classmethod
    def publish_event_message(cls, pubsub_client, from_target, to_target, event):
        topic = cls.get_event_message_topic(from_target, to_target)
        message = Dispatcher.Message.Event.new_data(**{
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION,
            },
            "body": {
                "target": to_target,
                "event": event
            }
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_change_vehicle_schedule_event_message(cls, pubsub_client, target_dispatcher, target_vehicle):
        event = Event.new_event(
            targets=[target_dispatcher, target_vehicle],
            name=Dispatcher.CONST.EVENT.CHANGE_SCHEDULE
        )
        cls.publish_event_message(pubsub_client, target_dispatcher, target_vehicle, event)

    @classmethod
    def publish_generated_vehicle_schedule(cls, pubsub_client, kvs_client, target_dispatcher, target_vehicle):
        generated_vehicle_schedule = Hook.get_generated_schedule(kvs_client, target_dispatcher, target_vehicle)
        cls.publish_schedule_message(pubsub_client, target_dispatcher, target_vehicle, generated_vehicle_schedule)

    @classmethod
    def publish_vehicle_info_to_user_in_need(cls, pubsub_client, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return
        applied_schedule = Hook.get_applied_schedule(kvs_client, target_dispatcher, target_vehicle)
        if applied_schedule is None:
            return
        for target_user, user_status in map(
                lambda x: (x["target"], x["status"]),
                filter(lambda y: y["status"].state == User.CONST.STATE.CALLING, dispatcher_status.user_statuses)):
            if any(filter(lambda e: Target.target_in_targets(target_user, e.targets), applied_schedule.events)):
                Hook.generate_vehicle_info_for_user(kvs_client, target_dispatcher, target_vehicle, target_user)
                cls.publish_vehicle_info_message(pubsub_client, kvs_client, target_dispatcher, target_user)
        return

    @classmethod
    def publish_get_on_to_user(cls, pubsub_client, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return
        vehicle_status = dispatcher_status.vehicle_status
        if vehicle_status is None:
            return
        target_user = Target.new_target(*vehicle_status.event_id.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[2:4])
        cls.publish_event_message(pubsub_client, target_dispatcher, target_user, Event.new_event(
            name=Dispatcher.CONST.EVENT.GET_ON,
            targets=[target_vehicle]
        ))

    @classmethod
    def publish_get_off_to_user(cls, pubsub_client, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return
        vehicle_status = dispatcher_status.vehicle_status
        if vehicle_status is None:
            return
        target_user = Target.new_target(*vehicle_status.event_id.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[2:4])
        cls.publish_event_message(pubsub_client, target_dispatcher, target_user, Event.new_event(
            name=Dispatcher.CONST.EVENT.GET_OFF,
            targets=[target_vehicle]
        ))

    @classmethod
    def publish_shift_event(cls, pubsub_client, from_target, to_target):
        cls.publish_event_message(
            pubsub_client, from_target, to_target, Event.new_event(
                targets=[from_target, to_target],
                name=Vehicle.CONST.EVENT.SHIFT_EVENT
            ))

    @classmethod
    def publish_vehicle_info_message(cls, pubsub_client, kvs_client, from_target, to_target):
        vehicle_info = Hook.get_vehicle_info(kvs_client, from_target, sub_target=to_target)
        if vehicle_info is None:
            return
        topic = cls.get_vehicle_info_message_topic(from_target, to_target)
        message = Dispatcher.Message.VehicleInfo.new_data(**{
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION
            },
            "body": vehicle_info
        })
        pubsub_client.publish(topic, message)

    @classmethod
    def publish_vehicle_info_message_to_user(cls, pubsub_client, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, target_vehicle)
        if dispatcher_status is None:
            return
        for target_user in map(lambda x: x["target"], dispatcher_status.user_statuses):
            cls.publish_vehicle_info_message(pubsub_client, kvs_client, target_dispatcher, target_user)

    @classmethod
    def publish_user_engage(cls, pubsub_client, kvs_client, target_user, structure, identifiable):
        status = Hook.get_status(kvs_client, target_user, User.Status)
        if status is None:
            return
        if status.vehicle_info is None:
            return
        target_autoware = Target.new_target(Autoware.CONST.NODE_NAME, status.vehicle_info.target.id)
        if identifiable:
            rostopic = cls.get_state_cmd_rostopic(target_autoware)
        else:
            rostopic = cls.get_state_cmd_rostopic()
        pubsub_client.publish(rostopic, {"data": "engage"}, structure=structure, wait=True)
