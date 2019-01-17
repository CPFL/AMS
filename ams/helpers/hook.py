#!/usr/bin/env python
# coding: utf-8

from math import modf

from ams import VERSION, logger
from ams.helpers import Target, Event, Route, Simulator, Location
from ams.structures import (
    CLIENT, MessageHeader, Pose, RoutePoint, Schedule,
    Autoware, AutowareInterface, Vehicle, Dispatcher, TrafficSignal, User)


class Hook(object):

    @classmethod
    def get_target_from_key(cls, key):
        group, _id = key.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[0:2]
        return Target.new_target(group, _id)

    @classmethod
    def get_config_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.encode(target),
            "config"])

    @classmethod
    def get_status_key(cls, target, sub_target=None):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [Target.encode(target)] +
            ([] if sub_target is None else [Target.encode(sub_target)]) +
            ["status"])

    @classmethod
    def get_vehicle_info_key(cls, target, sub_target=None):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [Target.encode(target)] +
            ([] if sub_target is None else [Target.encode(sub_target)]) +
            ["vehicle_info"])

    @classmethod
    def get_vehicle_location_key(cls, target):
        return Target.encode(target) + Autoware.CONST.TOPIC.VEHICLE_LOCATION

    @classmethod
    def get_current_pose_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.encode(target),
            ] + AutowareInterface.CONST.TOPIC.CATEGORIES.CURRENT_POSE)

    @classmethod
    def get_received_lane_array_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            cls.get_lane_array_key(target),
            "received"
        ])

    @classmethod
    def get_lane_array_key(cls, target):
        return Target.encode(target) + AutowareInterface.CONST.TOPIC.LANE_ARRAY

    @classmethod
    def get_decision_maker_state_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.encode(target),
            ] + AutowareInterface.CONST.TOPIC.CATEGORIES.DECISION_MAKER_STATE)

    @classmethod
    def get_state_cmd_key(cls, target):
        return Target.encode(target) + AutowareInterface.CONST.TOPIC.STATE_CMD

    @classmethod
    def get_route_point_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.encode(target),
            ] + AutowareInterface.CONST.TOPIC.CATEGORIES.ROUTE_POINT)

    @classmethod
    def get_stop_waypoint_index_key(cls, target):
        return Target.encode(target) + AutowareInterface.CONST.TOPIC.STOP_WAYPOINT_INDEX

    @classmethod
    def get_schedule_key(cls, target, sub_target=None):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [Target.encode(target)] +
            ([] if sub_target is None else [Target.encode(sub_target)]) +
            ["schedule"])

    @classmethod
    def get_received_schedule_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            cls.get_schedule_key(target),
            "received"
        ])

    @classmethod
    def get_applied_schedule_key(cls, from_target, to_target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            cls.get_schedule_key(from_target, sub_target=to_target),
            "applied"
        ])

    @classmethod
    def get_generated_schedule_key(cls, from_target, to_target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            cls.get_schedule_key(from_target, sub_target=to_target),
            "generated"
        ])

    @classmethod
    def get_event_key(cls, target, sub_target=None):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [Target.encode(target)] +
            ([] if sub_target is None else [Target.encode(sub_target)]) +
            ["event"])

    @classmethod
    def get_received_stop_signal_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.encode(target),
            "stop_signal"
        ])

    @classmethod
    def get_vehicle_status_key(cls, target_dispatcher, target_vehicle):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.encode(target_dispatcher),
            Target.encode(target_vehicle),
            "vehicle_status"
        ])

    @classmethod
    def get_traffic_signal_statuses_key(cls, target_dispatcher, target_vehicle):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.encode(target_dispatcher),
            Target.encode(target_vehicle),
            "traffic_signal_statuses"
        ])

    @classmethod
    def get_user_statuses_key(cls, target_dispatcher, target_vehicle):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.encode(target_dispatcher),
            Target.encode(target_vehicle),
            "user_statuses"
        ])

    @classmethod
    def get_relation_key(cls, target_owner, relation_name, from_id, to_id):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.encode(target_owner), relation_name, from_id, to_id])

    @classmethod
    def get_relation_keys(cls, kvs_client, target_owner, relation_name, from_id=None):
        if from_id is not None:
            return kvs_client.keys(
                CLIENT.KVS.KEY_PATTERN_DELIMITER.join([Target.encode(target_owner), relation_name, from_id, "*"]))
        else:
            return kvs_client.keys(
                CLIENT.KVS.KEY_PATTERN_DELIMITER.join([Target.encode(target_owner), relation_name, "*"]))

    @classmethod
    def get_config(cls, kvs_client, target, structure=None):
        key = cls.get_config_key(target)
        value = kvs_client.get(key)
        if structure is not None and value.__class__.__name__ == "dict":
            value = structure.new_data(**value)
        return value

    @classmethod
    def get_status(cls, kvs_client, target, structure=None, sub_target=None):
        key = cls.get_status_key(target, sub_target)
        value = kvs_client.get(key)
        if structure is not None and value.__class__.__name__ == "dict":
            value = structure.new_data(**value)
        return value

    @classmethod
    def get_schedule(cls, kvs_client, target, sub_target=None):
        key = cls.get_schedule_key(target, sub_target)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = Dispatcher.Schedule.new_data(**value)
        return value

    @classmethod
    def get_event(cls, kvs_client, target, sub_target=None):
        key = cls.get_event_key(target, sub_target)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = Event.Structure.new_data(**value)
        return value

    @classmethod
    def get_received_schedule(cls, kvs_client, target):
        key = cls.get_received_schedule_key(target)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = Dispatcher.Schedule.new_data(**value)
        return value

    @classmethod
    def get_applied_schedule(cls, kvs_client, from_target, to_target):
        key = cls.get_applied_schedule_key(from_target, to_target)
        value = kvs_client.get(key)
        if value is not None:
            value = Dispatcher.Schedule.new_data(**value)
        return value

    @classmethod
    def get_generated_schedule(cls, kvs_client, from_target, to_target):
        key = cls.get_generated_schedule_key(from_target, to_target)
        value = kvs_client.get(key)
        if value is not None:
            value = Dispatcher.Schedule.new_data(**value)
        return value

    @classmethod
    def get_received_stop_signal(cls, kvs_client, target):
        key = cls.get_received_stop_signal_key(target)
        value = kvs_client.get(key)
        return value

    @classmethod
    def get_vehicle_status(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_vehicle_status_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value is not None:
            value = Vehicle.Status.new_data(**value)
        return value

    @classmethod
    def get_traffic_signal_statuses(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_traffic_signal_statuses_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value is not None:
            value = list(map(lambda x: TrafficSignal.Status.new_data(**x), value))
        return value

    @classmethod
    def get_user_statuses(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_user_statuses_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value is not None:
            value = list(map(lambda x: {
                "target": Target.Structure.new_data(**x["target"]),
                "status": User.Status.new_data(**x["status"])
            }, value))
        return value

    @classmethod
    def get_response_config_message(cls, kvs_client, target, request_message):
        return {
            "header": MessageHeader.new_data(**{
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION,
                "request_id": request_message.header.id
            }),
            "body": cls.get_config(kvs_client, target)
        }

    @classmethod
    def get_response_status_message(cls, kvs_client, target, request_message):
        return {
            "header": MessageHeader.new_data(**{
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION,
                "request_id": request_message.header.id
            }),
            "body": cls.get_status(kvs_client, target)
        }

    @classmethod
    def get_route_code_from_lane_array_id(cls, kvs_client, target, lane_array_id):
        relation_keys = cls.get_relation_keys(
            kvs_client, target, AutowareInterface.CONST.KEY_PARTS.LANE_ARRAY_ID_ROUTE_CODE)
        route_code = None
        for relation_key in relation_keys:
            relation_key_lane_array_id, relation_key_route_code = relation_key.split(
                CLIENT.KVS.KEY_PATTERN_DELIMITER)[3:5]
            if relation_key_lane_array_id == str(lane_array_id):
                route_code = relation_key_route_code
        return route_code

    @classmethod
    def delete_disuse_route_code_lane_array_id_relations(cls, kvs_client, target, lane_array_id, route_code):
        lane_array_id_relation_keys = cls.get_relation_keys(
            kvs_client, target, AutowareInterface.CONST.KEY_PARTS.LANE_ARRAY_ID_ROUTE_CODE)
        route_code_relation_keys = cls.get_relation_keys(
            kvs_client, target, AutowareInterface.CONST.KEY_PARTS.ROUTE_CODE_LANE_ARRAY_ID)
        current_relation = cls.get_relation_key(
            target, AutowareInterface.CONST.KEY_PARTS.ROUTE_CODE_LANE_ARRAY_ID, route_code, str(lane_array_id))

        delete_keys = []
        if current_relation in route_code_relation_keys:
            delete_keys.append(current_relation)

        for relation_key in lane_array_id_relation_keys:
            relation_key_lane_array_id, relation_key_route_code = relation_key.split(
                CLIENT.KVS.KEY_PATTERN_DELIMITER)[3:5]
            if relation_key_route_code == route_code and relation_key_lane_array_id != str(lane_array_id):
                delete_keys.append(relation_key)
            else:
                if relation_key_route_code != route_code:
                    route_code_relation_key = cls.get_relation_key(
                        target, AutowareInterface.CONST.KEY_PARTS.ROUTE_CODE_LANE_ARRAY_ID,
                        relation_key_route_code, relation_key_lane_array_id)
                    if route_code_relation_key not in route_code_relation_keys:
                        delete_keys.append(relation_key)

        for relation_key in route_code_relation_keys:
            relation_key_route_code, relation_key_lane_array_id = relation_key.split(
                CLIENT.KVS.KEY_PATTERN_DELIMITER)[3:5]
            if relation_key_route_code == route_code and relation_key_lane_array_id != str(lane_array_id):
                delete_keys.append(relation_key)

        for delete_key in delete_keys:
            kvs_client.delete(delete_key)

    @classmethod
    def set_config(cls, kvs_client, target, value, get_key=None, timestamp_string=None):
        key = cls.get_config_key(target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_status(cls, kvs_client, target, value, get_key=None, timestamp_string=None, sub_target=None):
        key = cls.get_status_key(target, sub_target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_schedule(cls, kvs_client, target, value, get_key=None, timestamp_string=None, sub_target=None):
        key = cls.get_schedule_key(target, sub_target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_event(cls, kvs_client, target, value, get_key=None, timestamp_string=None, sub_target=None):
        key = cls.get_event_key(target, sub_target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_received_schedule(cls, kvs_client, target, value, get_key=None, timestamp_string=None):
        key = cls.get_received_schedule_key(target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_applied_schedule(cls, kvs_client, from_target, to_target, value, get_key=None, timestamp_string=None):
        key = cls.get_applied_schedule_key(from_target, to_target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_generated_schedule(
            cls, kvs_client, from_target, to_target, value, get_key=None, timestamp_string=None):
        key = cls.get_generated_schedule_key(from_target, to_target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_received_stop_signal(cls, kvs_client, target, value, get_key=None, timestamp_string=None):
        key = cls.get_received_stop_signal_key(target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_vehicle_info(cls, kvs_client, target, value, timestamp_string=None, sub_target=None):
        key = cls.get_vehicle_info_key(target, sub_target)
        return kvs_client.set(key, value, timestamp_string=timestamp_string)

    @classmethod
    def set_vehicle_location(cls, kvs_client, target, value):
        key = cls.get_vehicle_location_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_route_point(cls, kvs_client, target, value):
        key = cls.get_route_point_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_current_pose(cls, kvs_client, target, value):
        key = cls.get_current_pose_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_state_cmd(cls, kvs_client, target, value):
        key = cls.get_state_cmd_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_received_lane_array(cls, kvs_client, target, value):
        key = cls.get_received_lane_array_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_lane_array(cls, kvs_client, target, value):
        key = cls.get_lane_array_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_decision_maker_state(cls, kvs_client, target, value):
        key = cls.get_decision_maker_state_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_stop_waypoint_index(cls, kvs_client, target, value):
        key = cls.get_stop_waypoint_index_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_vehicle_status(cls, kvs_client, target_dispatcher, target_vehicle, value, timestamp_string):
        key = cls.get_vehicle_status_key(target_dispatcher, target_vehicle)
        return kvs_client.set(key, value, timestamp_string=timestamp_string)

    @classmethod
    def set_traffic_signal_statuses(cls, kvs_client, target_dispatcher, target_vehicle, value):
        key = cls.get_traffic_signal_statuses_key(target_dispatcher, target_vehicle)
        return kvs_client.set(key, value)

    @classmethod
    def set_user_statuses(cls, kvs_client, target_dispatcher, target_vehicle, value):
        key = cls.get_user_statuses_key(target_dispatcher, target_vehicle)
        return kvs_client.set(key, value)

    @classmethod
    def set_relation(cls, kvs_client, target_owner, relation_name, from_id, to_id):
        key = cls.get_relation_key(target_owner, relation_name, from_id, to_id)
        return kvs_client.set(key, None)

    @classmethod
    def set_route_code_lane_array_id_relation(cls, kvs_client, target, route_code, lane_array_id):
        set_flag = cls.set_relation(
            kvs_client, target, AutowareInterface.CONST.KEY_PARTS.LANE_ARRAY_ID_ROUTE_CODE,
            str(lane_array_id), route_code)
        set_flag *= cls.set_relation(
            kvs_client, target, AutowareInterface.CONST.KEY_PARTS.ROUTE_CODE_LANE_ARRAY_ID,
            route_code, str(lane_array_id))
        return set_flag

    @classmethod
    def initialize_vehicle_location(cls, kvs_client, target):
        lane_array = cls.get_lane_array(kvs_client, target)
        if lane_array is not None:
            nsec, sec = modf(Event.get_time())
            return cls.set_vehicle_location(kvs_client, target, Autoware.Status.VehicleLocation.new_data(**{
                "header": {
                    "seq": 0,
                    "stamp": {
                        "secs": int(sec),
                        "nsecs": int(nsec * (10 ** 9))
                    },
                    "frame_id": ""
                },
                "lane_array_id": lane_array["id"],
                "waypoint_index": 0
            }))
        return False

    @classmethod
    def initialize_state_cmd(cls, kvs_client, target):
        return cls.set_state_cmd(kvs_client, target, None)

    @classmethod
    def initialize_lane_array(cls, kvs_client, target):
        if cls.set_lane_array(kvs_client, target, None):
            return cls.initialize_received_lane_array(kvs_client, target)
        return False

    @classmethod
    def initialize_received_lane_array(cls, kvs_client, target):
        return cls.set_received_lane_array(kvs_client, target, None)

    @classmethod
    def initialize_vehicle_schedule(cls, kvs_client, target_vehicle):
        return cls.set_schedule(kvs_client, target_vehicle, None)

    @classmethod
    def initialize_received_schedule(cls, kvs_client, target_vehicle):
        return cls.set_received_schedule(kvs_client, target_vehicle, None)

    @classmethod
    def initialize_vehicle_received_schedule(cls, kvs_client, target_vehicle):
        return cls.initialize_received_schedule(kvs_client, target_vehicle)

    @classmethod
    def initialize_applied_schedule(cls, kvs_client, from_target, to_target):
        return cls.set_applied_schedule(kvs_client, from_target, to_target, None)

    @classmethod
    def initialize_generated_schedule(cls, kvs_client, from_target, to_target):
        return cls.set_generated_schedule(kvs_client, from_target, to_target, None)

    @classmethod
    def initialize_event(cls, kvs_client, target, sub_target=None):
        return cls.set_event(kvs_client, target, None, sub_target=sub_target)

    @classmethod
    def initialize_vehicle_event(cls, kvs_client, target_vehicle):
        return cls.initialize_event(kvs_client, target_vehicle)

    @classmethod
    def initialize_dispatcher_event(cls, kvs_client, target_dispatcher, target_vehicle):
        return cls.initialize_event(kvs_client, target_dispatcher, sub_target=target_vehicle)

    @classmethod
    def initialize_received_stop_signal(cls, kvs_client, target):
        return cls.set_received_stop_signal(kvs_client, target, None)

    @classmethod
    def initialize_vehicle_info(cls, kvs_client, target, sub_target=None):
        cls.set_vehicle_info(kvs_client, target, None, sub_target=sub_target)

    @classmethod
    def get_vehicle_info(cls, kvs_client, target, sub_target=None):
        key = cls.get_vehicle_info_key(target, sub_target)
        value = kvs_client.get(key)
        if value is not None:
            value = Vehicle.Info.new_data(**value)
        return value

    @classmethod
    def get_vehicle_location(cls, kvs_client, target):
        key = cls.get_vehicle_location_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.VehicleLocation.new_data(**value)
        return value

    @classmethod
    def get_current_pose(cls, kvs_client, target):
        key = cls.get_current_pose_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.CurrentPose.new_data(**value)
        return value

    @classmethod
    def get_route_point(cls, kvs_client, target):
        key = cls.get_route_point_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = RoutePoint.new_data(**value)
        return value

    @classmethod
    def get_state_cmd(cls, kvs_client, target):
        key = cls.get_state_cmd_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.StateCMD.new_data(**value)
        return value

    @classmethod
    def get_received_lane_array(cls, kvs_client, target):
        key = cls.get_received_lane_array_key(target)
        value = kvs_client.get(key)
        return value

    @classmethod
    def get_lane_array(cls, kvs_client, target):
        key = cls.get_lane_array_key(target)
        value = kvs_client.get(key)
        return value

    @classmethod
    def get_decision_maker_state(cls, kvs_client, target):
        key = cls.get_decision_maker_state_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.DecisionMakerState.new_data(**value)
        return value

    @classmethod
    def get_stop_waypoint_index(cls, kvs_client, target):
        key = cls.get_stop_waypoint_index_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.StopWaypointIndex.new_data(**value)
        return value

    @classmethod
    def get_current_vehicle_event(cls, vehicle_status, vehicle_schedule):
        return Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)

    @classmethod
    def get_vehicle_event_names(cls, vehicle_schedule):
        return list(map(lambda x: x.name, vehicle_schedule.events))

    @classmethod
    def get_next_vehicle_event_index(cls, vehicle_status, vehicle_schedule):
        next_vehicle_event = Event.get_next_event_by_current_event_id(
            vehicle_schedule.events, vehicle_status.event_id)
        return vehicle_schedule.events.index(next_vehicle_event)

    @classmethod
    def get_next_vehicle_event_id(cls, vehicle_status, vehicle_schedule):
        return Event.get_next_event_by_current_event_id(vehicle_schedule.events, vehicle_status.event_id).id

    @classmethod
    def update_next_vehicle_event_end_time_with_current_time_and_duration(cls, vehicle_status, vehicle_schedule):
        current_time = Event.get_time()
        next_vehicle_event_index = cls.get_next_vehicle_event_index(vehicle_status, vehicle_schedule)
        duration = vehicle_schedule.events[next_vehicle_event_index].period.end - \
            vehicle_schedule.events[next_vehicle_event_index].period.start
        vehicle_schedule.events[next_vehicle_event_index].period.start = current_time
        vehicle_schedule.events[next_vehicle_event_index].period.end = current_time + duration

    @classmethod
    def generate_lane_array_from_route_code(cls, maps_client, route_code):
        return maps_client.route.generate_lane_array(route_code)

    @classmethod
    def generate_pose_from_current_pose(cls, current_pose):
        return Pose.new_data(
            position=Pose.Position.new_data(**current_pose.pose.position),
            orientation=Pose.Orientation.new_data(
                quaternion=Pose.Orientation.Quaternion.new_data(**current_pose.pose.orientation),
                rpy=None
            )
        )

    @classmethod
    def generate_location_from_route_point(cls, maps_client, route_point):
        route = Route.decode(route_point.route_code)
        locations = maps_client.route.get_locations(route)
        return locations[route_point.index]

    @classmethod
    def generate_pose_from_location(cls, maps_client, location):
        return maps_client.lane.get_pose(location.lane_code, location.waypoint_id)

    @classmethod
    def generate_state_cmd_from_data(cls, data):
        return Autoware.Status.StateCMD.new_data(data=data)

    @classmethod
    def generate_vehicle_schedule_from_config(cls, kvs_client, target_dispatcher, target_vehicle, schedule_id):
        config = cls.get_config(kvs_client, target_dispatcher, Dispatcher.Config)
        schedule = list(filter(lambda s: s.id == schedule_id, config.vehicle_schedules))[0]
        events = []
        start_time = Event.get_time()
        for event in schedule.events:
            events.append(Event.new_event(
                targets=[target_vehicle],
                _id=event.id if "id" in event else None,
                name=event.name,
                start_time=start_time,
                end_time=start_time + event.duration if "duration" in event else 0,
                route_code=event.route_code if "route_code" in event else None
            ))
            start_time += event.duration if "duration" in event else 0
        generated_schedule = Schedule.new_data(**{
            "id": schedule_id,
            "events": events
        })
        cls.set_generated_schedule(kvs_client, target_dispatcher, target_vehicle, generated_schedule)

    @classmethod
    def generate_vehicle_schedule_from_user_statuses(cls, kvs_client, maps_client, target_dispatcher, target_vehicle):
        dispatcher_status = cls.get_status(kvs_client, target_dispatcher, Dispatcher.Status, target_vehicle)
        if dispatcher_status is None:
            logger.warning("dispatcher_status is None")
            return
        if None in [dispatcher_status.vehicle_status, dispatcher_status.user_statuses]:
            logger.warning("None in [dispatcher_status.vehicle_status, dispatcher_status.user_statuses]")
            return
        if dispatcher_status.vehicle_status.location is None:
            logger.warning("dispatcher_status.vehicle_status.location is None")
            return
        applied_vehicle_schedule = cls.get_applied_schedule(kvs_client, target_dispatcher, target_vehicle)

        generated_vehicle_schedule_events = []
        if applied_vehicle_schedule is not None:
            index = 0
            vehicle_event_id = dispatcher_status.vehicle_status.event_id
            if vehicle_event_id is not None:
                index = Event.get_event_index_by_event_id(applied_vehicle_schedule.events, vehicle_event_id)
            applied_vehicle_schedule_without_old_events = applied_vehicle_schedule.events[index:]
            generated_vehicle_schedule_events.extend(applied_vehicle_schedule_without_old_events)

        if applied_vehicle_schedule is None:
            user_statuses = dispatcher_status.user_statuses
        else:
            user_statuses = list(filter(
                lambda us: not any(filter(
                    lambda e: Target.target_in_targets(us["target"], e.targets),
                    applied_vehicle_schedule.events)),
                dispatcher_status.user_statuses))
        if 0 < len(user_statuses):
            vehicle_location = None
            if applied_vehicle_schedule is not None:
                filtered_events = list(filter(
                    lambda x: "route_code" in x and x.route_code is not None, applied_vehicle_schedule.events))
                if 0 < len(filtered_events):
                    final_route = Route.decode(filtered_events[-1].route_code)
                    vehicle_location = Location.Structure.new_data(**{
                        "waypoint_id": final_route.waypoint_ids[-1],
                        "lane_code": final_route.lane_codes[-1]
                    })
            if vehicle_location is None:
                if dispatcher_status.vehicle_status.route_point is not None:
                    latest_lane_code_waypoint_id_relation = maps_client.route.generate_lane_code_waypoint_id_relations(
                        dispatcher_status.vehicle_status.route_point.route_code)[-1]
                    vehicle_location = Location.Structure.new_data(**{
                        "lane_code": latest_lane_code_waypoint_id_relation["lane_code"],
                        "waypoint_id": latest_lane_code_waypoint_id_relation["waypoint_ids"][-1]
                    })
            if vehicle_location is None:
                vehicle_location = dispatcher_status.vehicle_status.location

            if vehicle_location is None:
                return

            target_user = user_statuses[0]["target"]
            user_status = user_statuses[0]["status"]
            user_start_location = Location.Structure.new_data(**{
                "waypoint_id": user_status.start_location.waypoint_id,
                "lane_code": user_status.start_location.lane_code
            })
            user_goal_location = Location.Structure.new_data(**{
                "waypoint_id": user_status.goal_location.waypoint_id,
                "lane_code": user_status.goal_location.lane_code
            })

            locations = [vehicle_location]
            if not Location.same_locations(vehicle_location, user_start_location):
                locations.append(user_start_location)
            locations.append(user_goal_location)

            route_array = maps_client.route.search_multi_destinations_shortest_route_array(locations)

            target_user_code = Target.encode(target_user)
            route_codes = list(map(lambda x: Route.encode(x), route_array))
            if not Location.same_locations(vehicle_location, user_start_location):
                generated_vehicle_schedule_events.extend([
                    Event.new_event(
                        targets=[],
                        name=Vehicle.CONST.EVENT.SEND_LANE_ARRAY,
                        route_code=route_codes.pop(0)),
                    Event.new_event(
                        targets=[],
                        name=Vehicle.CONST.EVENT.SEND_ENGAGE),
                    Event.new_event(
                        _id=Vehicle.CONST.EVENT_ID_PARTS.DELIMITER.join([
                            Vehicle.CONST.EVENT_ID_PARTS.AFTER_MOVE_TO_USER_START,
                            Vehicle.CONST.EVENT.SEND_GOTO_WAIT_ORDER,
                            target_user_code]),
                        targets=[],
                        name=Vehicle.CONST.EVENT.SEND_GOTO_WAIT_ORDER)])
            generated_vehicle_schedule_events.extend([
                Event.new_event(
                    _id=Vehicle.CONST.EVENT_ID_PARTS.DELIMITER.join([
                        Vehicle.CONST.EVENT_ID_PARTS.WAIT_AT_USER_START,
                        Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT,
                        target_user_code]),
                    targets=[target_user],
                    name=Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT),
                Event.new_event(
                    _id=Vehicle.CONST.EVENT_ID_PARTS.DELIMITER.join([
                        Vehicle.CONST.EVENT_ID_PARTS.WAIT_AT_USER_START,
                        Vehicle.CONST.EVENT.SEND_LANE_ARRAY,
                        target_user_code]),
                    targets=[target_user],
                    name=Vehicle.CONST.EVENT.SEND_LANE_ARRAY,
                    route_code=route_codes.pop(0)),
                Event.new_event(
                    _id=Vehicle.CONST.EVENT_ID_PARTS.DELIMITER.join([
                        Vehicle.CONST.EVENT_ID_PARTS.AFTER_MOVE_TO_USER_GOAL,
                        Vehicle.CONST.EVENT.SEND_GOTO_WAIT_ORDER,
                        target_user_code]),
                    targets=[target_user],
                    name=Vehicle.CONST.EVENT.SEND_GOTO_WAIT_ORDER),
                Event.new_event(
                    _id=Vehicle.CONST.EVENT_ID_PARTS.DELIMITER.join([
                        Vehicle.CONST.EVENT_ID_PARTS.WAIT_AT_USER_GOAL,
                        Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT,
                        target_user_code]),
                    targets=[target_user],
                    name=Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT),
            ])

        generated_vehicle_schedule = Schedule.new_data(**{
            "id": Event.get_id(),
            "events": generated_vehicle_schedule_events
        })
        cls.set_generated_schedule(kvs_client, target_dispatcher, target_vehicle, generated_vehicle_schedule)

    @classmethod
    def generate_route_point(cls, kvs_client, target, vehicle_location):
        route_code = cls.get_route_code_from_lane_array_id(kvs_client, target, vehicle_location.lane_array_id)
        if route_code is not None:
            cls.delete_disuse_route_code_lane_array_id_relations(
                kvs_client, target, vehicle_location.lane_array_id, route_code)
            return RoutePoint.new_data(
                route_code=route_code,
                index=vehicle_location.waypoint_index
            )
        else:
            logger.warning(
                "cannot generate route_point for vehicle_location: {}".format(logger.pformat(vehicle_location)))
        return None

    @classmethod
    def generate_vehicle_stop_route_point(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        stop_signal = Hook.get_received_stop_signal(kvs_client, target_vehicle)
        if stop_signal is None or stop_signal:
            vehicle_schedule = Hook.get_schedule(kvs_client, target_vehicle)
            vehicle_event = Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)
            route_section = maps_client.route.generate_route_section_with_route_codes(
                vehicle_event.route_code, vehicle_status.route_point.route_code)
            route_point = RoutePoint.new_data(**{
                "route_code": route_section.route_code,
                "index": route_section.end_index
            })
        else:
            route_point = RoutePoint.new_data(**{
                "route_code": vehicle_status.route_point.route_code,
                "index": -1
            })
        return route_point

    @classmethod
    def remove_old_traffic_signal_event_from_schedule(cls, kvs_client, target):
        schedule = cls.get_schedule(kvs_client, target)
        if schedule is None:
            return
        current_time = Event.get_time()
        schedule.events = list(filter(
            lambda x: current_time < x.period.end,
            schedule.events
        ))[:]
        cls.set_schedule(kvs_client, target, schedule)

    @classmethod
    def append_traffic_signal_event_to_schedule_from_cycle(cls, kvs_client, target):
        config = cls.get_config(kvs_client, target, TrafficSignal.Config)
        schedule = cls.get_schedule(kvs_client, target)
        if schedule is None or len(schedule.events) == 0:
            event = Event.get_event_from_cycle([target], config.cycle, Event.get_time())
            schedule = Schedule.new_data(**{
                "id": Event.get_id(),
                "events": [event]
            })
        else:
            event = Event.get_event_from_cycle([target], config.cycle, schedule.events[-1].period.end)
            schedule.events.append(event)
        cls.set_schedule(kvs_client, target, schedule)

    @classmethod
    def replace_schedule(cls, kvs_client, target):
        received_schedule = cls.get_received_schedule(kvs_client, target)
        if received_schedule is not None:
            if cls.set_schedule(kvs_client, target, received_schedule):
                return cls.initialize_received_schedule(kvs_client, target)
        return False

    @classmethod
    def replace_applied_schedule(cls, kvs_client, target_dispatcher, target_vehicle):
        generated_schedule = cls.get_generated_schedule(kvs_client, target_dispatcher, target_vehicle)
        if generated_schedule is not None:
            if cls.set_applied_schedule(kvs_client, target_dispatcher, target_vehicle, generated_schedule):
                return cls.initialize_generated_schedule(kvs_client, target_dispatcher, target_vehicle)
        return False

    @classmethod
    def start_vehicle_schedule(cls, kvs_client, target_vehicle):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is None:
            return False
        vehicle_schedule = cls.get_schedule(kvs_client, target_vehicle)
        if vehicle_schedule is None:
            logger.warning("No vehicle schedule")
            return False
        if vehicle_status.schedule_id is not None:
            logger.warning("vehicle schedule id is not initialized")
            return False
        if vehicle_status.event_id is not None:
            logger.warning("vehicle event id is not initialized")
            return False
        vehicle_status.schedule_id = vehicle_schedule.id
        vehicle_status.event_id = vehicle_schedule.events[0].id
        return cls.set_status(kvs_client, target_vehicle, vehicle_status)

    @classmethod
    def restart_vehicle_schedule(cls, kvs_client, target_vehicle):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is None:
            return False
        vehicle_schedule = cls.get_schedule(kvs_client, target_vehicle)
        if vehicle_schedule is None:
            logger.warning("No vehicle schedule")
            return False
        vehicle_status.schedule_id = vehicle_schedule.id
        return cls.set_status(kvs_client, target_vehicle, vehicle_status)

    @classmethod
    def reset_vehicle_event_id(cls, kvs_client, target_vehicle, event_index=0):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is None:
            return False
        vehicle_schedule = cls.get_schedule(kvs_client, target_vehicle)
        if vehicle_schedule is None:
            logger.warning("No vehicle schedule")
            return False
        vehicle_status.event_id = vehicle_schedule.events[event_index].id
        return cls.set_status(kvs_client, target_vehicle, vehicle_status)

    @classmethod
    def start_vehicle_event(cls, kvs_client, target_vehicle):
        status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if status is None:
            return
        status.on_event = True
        cls.set_status(kvs_client, target_vehicle, status)

    @classmethod
    def suspend_vehicle_event(cls, kvs_client, target_vehicle):
        status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if status is None:
            return
        if status.on_event:
            status.on_event = False
            cls.set_status(kvs_client, target_vehicle, status)

    @classmethod
    def shift_to_next_vehicle_event(cls, kvs_client, target_vehicle):
        status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        schedule = cls.get_schedule(kvs_client, target_vehicle)
        if None in [status, schedule]:
            return
        status.on_event = False
        next_event = Event.get_next_event_by_current_event_id(schedule.events, status.event_id)
        if next_event is None:
            status.schedule_id = None
            status.event_id = None
        else:
            status.event_id = next_event.id
        cls.set_status(kvs_client, target_vehicle, status)

    @classmethod
    def end_vehicle_event(cls, kvs_client, target_vehicle):
        status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if status is None:
            return
        if status.on_event:
            status.on_event = False
            schedule = cls.get_schedule(kvs_client, target_vehicle)
            if schedule is None:
                return
            next_event = Event.get_next_event_by_current_event_id(schedule.events, status.event_id)
            if next_event is None:
                status.schedule_id = None
                status.event_id = None
            else:
                status.event_id = next_event.id
            cls.set_status(kvs_client, target_vehicle, status)

    @classmethod
    def update_vehicle_location(cls, kvs_client, target):
        config = cls.get_config(kvs_client, target, Autoware.Config)
        vehicle_location = cls.get_vehicle_location(kvs_client, target)
        current_pose = cls.get_current_pose(kvs_client, target)
        lane_array = cls.get_lane_array(kvs_client, target)
        if lane_array is not None:
            current_vehicle_location = Simulator.search_vehicle_location_from_lane_array(current_pose, lane_array)
            vehicle_location.lane_array_id = lane_array["id"]

            next_waypoint_index = min([
                current_vehicle_location.waypoint_index + config.step_size,
                len(lane_array["lanes"][0]["waypoints"]) - 1
            ])
            stop_waypoint_index = cls.get_stop_waypoint_index(kvs_client, target)
            if stop_waypoint_index is not None:
                if stop_waypoint_index.data != -1:
                    next_waypoint_index = min([
                        next_waypoint_index,
                        stop_waypoint_index.data
                    ])

            vehicle_location.waypoint_index = next_waypoint_index
            return cls.set_vehicle_location(kvs_client, target, vehicle_location)
        return False

    @classmethod
    def update_current_pose(cls, kvs_client, target):
        lane_array = cls.get_lane_array(kvs_client, target)
        if lane_array is not None:
            vehicle_location = cls.get_vehicle_location(kvs_client, target)
            if 0 <= vehicle_location.waypoint_index < len(lane_array["lanes"][0]["waypoints"]):
                return cls.set_current_pose(
                    kvs_client, target, lane_array["lanes"][0]["waypoints"][vehicle_location.waypoint_index]["pose"])
        return False

    @classmethod
    def update_lane_array(cls, kvs_client, target_autoware):
        received_lane_array = cls.get_received_lane_array(kvs_client, target_autoware)
        if received_lane_array is not None:
            lane_array = cls.get_lane_array(kvs_client, target_autoware)
            if received_lane_array != lane_array:
                cls.set_lane_array(kvs_client, target_autoware, received_lane_array)

    @classmethod
    def update_vehicle_route_point(cls, kvs_client, target_vehicle):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedule = cls.get_schedule(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedule]:
            vehicle_event = Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)
            if vehicle_event is not None:
                if "route_code" in vehicle_event:
                    vehicle_status.route_point = RoutePoint.new_data(**{
                        "route_code": vehicle_event.route_code,
                        "index": 0
                    })
                    return cls.set_status(kvs_client, target_vehicle, vehicle_status)
        return False

    @classmethod
    def update_and_set_vehicle_pose(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        # logger.info("update_and_set_vehicle_pose.vehicle_status: {}".format(vehicle_status))
        if vehicle_status is not None:
            if all([
                vehicle_status.route_point is not None,
                vehicle_status.decision_maker_state.data != Autoware.CONST.DECISION_MAKER_STATE.WAIT_ORDER
            ]):
                # on vehicle_location_ros_message
                pose, location = maps_client.route.get_route_point_pose_and_location(vehicle_status.route_point)
                if None in [pose, location]:
                    logger.warning("Vehicle location is out of route.")
                else:
                    vehicle_status.location = location
                    vehicle_status.pose = pose
                    cls.set_status(kvs_client, target_vehicle, vehicle_status)

            else:
                # on current_pose_ros_message
                if vehicle_status.location is None and vehicle_status.current_pose is not None:
                    vehicle_status.location = maps_client.map_match.get_matched_location_on_lanes(
                        cls.generate_pose_from_current_pose(vehicle_status.current_pose))

                    if vehicle_status.location is not None:
                        vehicle_status.pose = cls.generate_pose_from_location(maps_client, vehicle_status.location)

                        cls.set_status(kvs_client, target_vehicle, vehicle_status)
                    else:
                        logger.warning("Current pose is out of range.")

    @classmethod
    def update_traffic_signal_light_color(cls, kvs_client, target):
        status = Hook.get_status(kvs_client, target, TrafficSignal.Status)
        schedule = Hook.get_schedule(kvs_client, target)
        if schedule is not None:
            event = Event.get_event_by_specified_time(schedule.events, Event.get_time())
            if status.light_color != event.name:
                status.light_color = event.name
                status.next_light_color = None
                status.next_update_time = None
                Hook.set_status(kvs_client, target, status)

    @classmethod
    def update_traffic_signal_next_light_color(cls, kvs_client, target_traffic_signal):
        status = Hook.get_status(kvs_client, target_traffic_signal, TrafficSignal.Status)
        schedule = Hook.get_schedule(kvs_client, target_traffic_signal)
        next_event = Event.get_next_event_by_specified_time(schedule.events, Event.get_time())
        if next_event is not None:
            status.next_light_color = next_event.name
            status.next_update_time = next_event.period.start
            Hook.set_status(kvs_client, target_traffic_signal, status)

    @classmethod
    def delete_config(cls, kvs_client, target):
        key = cls.get_config_key(target)
        kvs_client.delete(key)
        return True

    @classmethod
    def set_user_goal_at_random(cls, kvs_client, target, goal_location_candidates):
        import random
        goal_location = random.choice(goal_location_candidates)
        status = cls.get_status(kvs_client, target, User.Status)
        status.goal_location = goal_location
        cls.set_status(kvs_client, target, status)

    @classmethod
    def get_traffic_signal_status_keys(cls, kvs_client):
        key_pattern = cls.get_status_key(Target.new_target(TrafficSignal.CONST.NODE_NAME))
        return kvs_client.keys(key_pattern)

    @classmethod
    def get_traffic_signal_statuses_on_vehicle_route(cls, kvs_client, maps_client, target_dispatcher, target_vehicle):
        traffic_signal_status_keys = cls.get_traffic_signal_status_keys(kvs_client)

        filtered_traffic_signal_status_keys = []
        vehicle_status = cls.get_vehicle_status(kvs_client, target_dispatcher, target_vehicle)
        if vehicle_status is not None:
            if vehicle_status.route_point is not None:
                for traffic_signal_status_key in traffic_signal_status_keys:
                    route_code = cls.get_target_from_key(traffic_signal_status_key).id
                    if maps_client.route.route_code_in_route_code(route_code, vehicle_status.route_point.route_code):
                        if 0.0 < maps_client.route.calculate_distance_from_route_point_to_inner_route(
                                vehicle_status.route_point, route_code):
                            filtered_traffic_signal_status_keys.append(traffic_signal_status_key)

        traffic_statuses = []
        for traffic_signal_status_key in filtered_traffic_signal_status_keys:
            traffic_signal_status = cls.get_status(kvs_client, traffic_signal_status_key, TrafficSignal.Status)
            if traffic_signal_status is not None:
                traffic_statuses.append({
                    "target": cls.get_target_from_key(traffic_signal_status_key),
                    "status": traffic_signal_status
                })
        return traffic_statuses

    @classmethod
    def reduce_user_status(cls, kvs_client, target_user):
        status = cls.get_status(kvs_client, target_user, User.Status)
        vehicle_info = cls.get_vehicle_info(kvs_client, target_user)
        if None in [status, vehicle_info]:
            return
        status.vehicle_info = vehicle_info
        cls.set_status(kvs_client, target_user, status)

    @classmethod
    def reduce_dispatcher_status(cls, kvs_client, maps_client, target_dispatcher, target_vehicle):
        dispatcher_status = cls.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return False
        vehicle_status = cls.get_vehicle_status(kvs_client, target_dispatcher, target_vehicle)
        if vehicle_status is None:
            return False
        dispatcher_status.vehicle_status = vehicle_status
        if vehicle_status.route_point is not None:
            dispatcher_status.traffic_signal_statuses = cls.get_traffic_signal_statuses_on_vehicle_route(
                kvs_client, maps_client, target_dispatcher, target_vehicle)
        dispatcher_status.user_statuses = cls.get_user_statuses(kvs_client, target_dispatcher, target_vehicle)
        return Hook.set_status(kvs_client, target_dispatcher, dispatcher_status, sub_target=target_vehicle)

    @classmethod
    def relate_user_to_vehicle(cls, kvs_client, maps_client, target_dispatcher, target_user):
        user_status = cls.get_status(kvs_client, target_dispatcher, User.Status, sub_target=target_user)
        dispatcher_config = cls.get_config(kvs_client, target_dispatcher, Dispatcher.Config)

        goals = []
        goal_ext_costs = {}
        for target_vehicle in dispatcher_config.target_vehicles:
            applied_vehicle_schedule = cls.get_applied_schedule(kvs_client, target_dispatcher, target_vehicle)
            vehicle_status = cls.get_vehicle_status(kvs_client, target_dispatcher, target_vehicle)
            if vehicle_status is not None:
                if applied_vehicle_schedule is not None:
                    filtered_events = list(filter(
                        lambda x: "route_code" in x and x.route_code is not None, applied_vehicle_schedule.events))
                    if 0 < len(filtered_events):
                        final_route = Route.decode(filtered_events[-1].route_code)
                        goals.append({
                            "goal_id": target_vehicle.id,
                            "waypoint_id": final_route.waypoint_ids[-1],
                            "lane_code": final_route.lane_codes[-1]
                        })
                        # todo: estimate time of schedule end
                        goal_ext_costs[target_vehicle.id] = 600.0*(
                                len(applied_vehicle_schedule.events) - 1 -
                                Event.get_event_index_by_event_id(
                                    applied_vehicle_schedule.events, vehicle_status.event_id)
                        )
                else:
                    if vehicle_status.location is not None:
                        goals.append(vehicle_status.location)
                        goals[-1]["goal_id"] = target_vehicle.id
                        goal_ext_costs[target_vehicle.id] = 0.0

        shortest_routes = maps_client.route.search_shortest_routes(user_status.start_location, goals, reverse=True)
        if 0 == len(shortest_routes):
            logger.warning("There are no vehicles can be dispatched.")
            return

        shortest_route = min(shortest_routes.items(), key=lambda x: x[1]["cost"] + goal_ext_costs[x[1]["goal_id"]])[1]
        target_vehicle = Target.new_target(Vehicle.CONST.NODE_NAME, shortest_route["goal_id"])

        user_statuses = cls.get_user_statuses(kvs_client, target_dispatcher, target_vehicle)
        if user_statuses is None:
            return
        if not any(map(lambda x: Target.is_same(x["target"], target_user), user_statuses)):
            user_statuses.append({
                "target": target_user,
                "status": user_status
            })
        cls.set_user_statuses(kvs_client, target_dispatcher, target_vehicle, user_statuses)
        return

    @classmethod
    def add_user_status_to_user_statuses(cls, kvs_client, target_dispatcher, target_vehicle, target_user):
        user_status = cls.get_status(kvs_client, target_dispatcher, User.Status, target_user)
        user_statuses = cls.get_user_statuses(kvs_client, target_dispatcher, target_vehicle)
        if None in [user_status, user_statuses]:
            return
        if not any(map(lambda x: Target.is_same(x["target"], target_user), user_statuses)):
            return
        user_statuses = list(filter(lambda x: not Target.is_same(x["target"], target_user), user_statuses))
        user_statuses.append({
            "target": target_user,
            "status": user_status
        })
        cls.set_user_statuses(kvs_client, target_dispatcher, target_vehicle, user_statuses)
        return

    @classmethod
    def update_vehicle_info_for_user(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = cls.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return
        vehicle_status = dispatcher_status.vehicle_status
        if vehicle_status is None:
            return
        vehicle_event_id = vehicle_status.event_id
        if vehicle_event_id is None:
            return

        state = None
        current_target_user = None
        vehicle_event_id_parts = vehicle_event_id.split(Vehicle.CONST.EVENT_ID_PARTS.DELIMITER)
        if Vehicle.CONST.EVENT_ID_PARTS.AFTER_MOVE_TO_USER_START == vehicle_event_id_parts[0]:
            state = Vehicle.CONST.STATE.ON_ROUTE_TO_USER
        if Vehicle.CONST.EVENT_ID_PARTS.WAIT_AT_USER_START == vehicle_event_id_parts[0]:
            state = Vehicle.CONST.STATE.AT_USER_START_LOCATION
        if Vehicle.CONST.EVENT_ID_PARTS.AFTER_MOVE_TO_USER_GOAL == vehicle_event_id_parts[0]:
            if "\n" + Autoware.CONST.DECISION_MAKER_STATE.DRIVING not in vehicle_status.decision_maker_state.data:
                state = Vehicle.CONST.STATE.AT_USER_START_LOCATION
            else:
                state = Vehicle.CONST.STATE.ON_ROUTE_OF_USER
        if Vehicle.CONST.EVENT_ID_PARTS.WAIT_AT_USER_GOAL == vehicle_event_id_parts[0]:
            state = Vehicle.CONST.STATE.AT_USER_GOAL_LOCATION

        # logger.info("{} vehicle_info.state: {}".format(Target.encode(target_vehicle), state))
        if state is not None:
            location = vehicle_status.location
            current_target_user = Target.new_target(
                *vehicle_status.event_id.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[2:4])
            vehicle_info = Vehicle.Info.new_data(**{
                "target": target_vehicle,
                "state": state,
                "location": location
            })
            cls.set_vehicle_info(kvs_client, target_dispatcher, vehicle_info, sub_target=current_target_user)

        applied_schedule = cls.get_applied_schedule(kvs_client, target_dispatcher, target_vehicle)
        if applied_schedule is None:
            return
        event_index = Event.get_event_index_by_event_id(applied_schedule.events, vehicle_event_id)
        target_users = Event.generate_same_group_targets(applied_schedule.events[event_index:], User.CONST.NODE_NAME)
        if current_target_user is not None:
            target_users = list(filter(lambda x: not Target.is_same(current_target_user, x), target_users))
        for target_user in target_users:
            vehicle_info = Vehicle.Info.new_data(**{
                "target": target_vehicle,
                "state": None,
                "location": None
            })
            cls.set_vehicle_info(kvs_client, target_dispatcher, vehicle_info, sub_target=target_user)
        return

    @classmethod
    def initialize_status(cls, kvs_client, target, sub_target=None):
        return cls.set_status(kvs_client, target, None, sub_target=sub_target)

    @classmethod
    def initialize_user_status_in_transportation_finished(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = cls.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return None
        event_id = dispatcher_status.vehicle_status.event_id
        target_user = Target.new_target(*event_id.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[2:4])
        cls.initialize_status(kvs_client, target_dispatcher, sub_target=target_user)

    @classmethod
    def remove_transportation_finished_user_status_from_user_statuses(
            cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = cls.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return None
        event_id = dispatcher_status.vehicle_status.event_id
        target_user = Target.new_target(*event_id.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[2:4])

        user_statuses = cls.get_user_statuses(kvs_client, target_dispatcher, target_vehicle)
        filtered_user_statuses = list(filter(lambda x: not Target.is_same(target_user, x["target"]), user_statuses))
        if cls.set_user_statuses(kvs_client, target_dispatcher, target_vehicle, filtered_user_statuses):
            dispatcher_status.user_statuses = filtered_user_statuses
            cls.set_status(kvs_client, target_dispatcher, dispatcher_status, sub_target=target_vehicle)
