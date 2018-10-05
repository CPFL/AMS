#!/usr/bin/env python
# coding: utf-8

from time import time
from math import modf

from ams import VERSION, logger
from ams.helpers import Target, Schedule, Route, Simulator
from ams.structures import (
    CLIENT, MessageHeader, Pose, RoutePoint,
    EventLoop, Autoware, AutowareInterface, Vehicle, Dispatcher)


class Hook(object):

    @classmethod
    def get_config_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target),
            "config"])

    @classmethod
    def get_status_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target),
            "status"])

    @classmethod
    def get_closest_waypoint_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target),
                Autoware.CONST.TOPIC.CLOSEST_WAYPOINT
            ])

    @classmethod
    def get_current_pose_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target),
            ] + AutowareInterface.CONST.TOPIC.CATEGORIES.CURRENT_POSE)

    @classmethod
    def get_lane_array_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target),
            AutowareInterface.CONST.TOPIC.LANE_ARRAY
        ])

    @classmethod
    def get_decision_maker_state_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target),
            ] + AutowareInterface.CONST.TOPIC.CATEGORIES.DECISION_MAKER_STATE)

    @classmethod
    def get_state_cmd_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target),
            ] + Vehicle.CONST.TOPIC.CATEGORIES.STATE_CMD)

    @classmethod
    def get_light_color_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target),
            ] + Vehicle.CONST.TOPIC.CATEGORIES.LIGHT_COLOR)

    @classmethod
    def get_route_point_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target),
            ] + AutowareInterface.CONST.TOPIC.CATEGORIES.ROUTE_POINT)

    @classmethod
    def get_schedules_key(cls, target):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target),
            "schedules"])

    @classmethod
    def get_transportation_config_key(cls, target_dispatcher, target_vehicle):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_dispatcher),
                Target.get_code(target_vehicle)
            ] + Dispatcher.CONST.TOPIC.CATEGORIES.TRANSPORTATION_CONFIG
        )

    @classmethod
    def get_transportation_status_key(cls, target_dispatcher, target_vehicle):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_dispatcher),
                Target.get_code(target_vehicle)
            ] + Dispatcher.CONST.TOPIC.CATEGORIES.TRANSPORTATION_STATUS)

    @classmethod
    def get_relation_key(cls, target_owner, relation_name, from_id, to_id):
        return CLIENT.KVS.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_owner), relation_name, from_id, to_id])

    @classmethod
    def get_relation_keys(cls, kvs_client, target_owner, relation_name, from_id=None):
        if from_id is not None:
            return kvs_client.keys(
                CLIENT.KVS.KEY_PATTERN_DELIMITER.join([Target.get_code(target_owner), relation_name, from_id, "*"]))
        else:
            return kvs_client.keys(
                CLIENT.KVS.KEY_PATTERN_DELIMITER.join([Target.get_code(target_owner), relation_name, "*"]))

    @classmethod
    def get_config(cls, kvs_client, target, structure):
        key = cls.get_config_key(target)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = structure.new_data(**value)
        return value

    @classmethod
    def get_status(cls, kvs_client, target, structure):
        key = cls.get_status_key(target)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = structure.new_data(**value)
        return value

    @classmethod
    def get_schedules(cls, kvs_client, target):
        key = cls.get_schedules_key(target)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "list":
            value = Schedule.new_schedules(value)
        return value

    @classmethod
    def get_transportation_config(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_transportation_config_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = Dispatcher.TransportationConfig.new_data(**value)
        return value

    @classmethod
    def get_transportation_status(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_transportation_status_key(target_dispatcher, target_vehicle)
        value = kvs_client.get(key)
        if value.__class__.__name__ == "dict":
            value = Dispatcher.TransportationStatus.new_data(**value)
        return value

    @classmethod
    def get_response_config_message(cls, kvs_client, target, request_message):
        return {
            "header": MessageHeader.new_data(**{
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION,
                "request_id": request_message.header.id
            }),
            "body": cls.get_config(kvs_client, target, EventLoop.Config)
        }

    @classmethod
    def get_response_status_message(cls, kvs_client, target, request_message):
        return {
            "header": MessageHeader.new_data(**{
                "id": Schedule.get_id(),
                "time": Schedule.get_time(),
                "version": VERSION,
                "request_id": request_message.header.id
            }),
            "body": cls.get_status(kvs_client, target, EventLoop)
        }

    @classmethod
    def get_route_code_from_lane_array_id(cls, kvs_client, target, lane_array_id):
        relation_keys = cls.get_relation_keys(kvs_client, target, "route_code_lane_array_id")
        route_code = None
        for relation_key in relation_keys:
            relation_key_lane_array_id, relation_key_route_code = relation_key.split(
                CLIENT.KVS.KEY_PATTERN_DELIMITER)[3:5]
            if relation_key_lane_array_id == str(lane_array_id):
                route_code = relation_key_route_code
            else:
                kvs_client.delete(relation_key)
        return route_code

    @classmethod
    def set_config(cls, kvs_client, target, value, get_key=None, timestamp_string=None):
        key = cls.get_config_key(target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_status(cls, kvs_client, target, value, get_key=None, timestamp_string=None):
        key = cls.get_status_key(target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_schedules(cls, kvs_client, target, value, get_key=None, timestamp_string=None):
        key = cls.get_schedules_key(target)
        return kvs_client.set(key, value, get_key, timestamp_string)

    @classmethod
    def set_closest_waypoint(cls, kvs_client, target, value):
        key = cls.get_closest_waypoint_key(target)
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
    def set_lane_array(cls, kvs_client, target, value):
        key = cls.get_lane_array_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_decision_maker_state(cls, kvs_client, target, value):
        key = cls.get_decision_maker_state_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_light_color(cls, kvs_client, target, value):
        key = cls.get_light_color_key(target)
        return kvs_client.set(key, value)

    @classmethod
    def set_transportation_config(cls, kvs_client, target_dispatcher, target_vehicle, transportation_config):
        return kvs_client.set(
            cls.get_transportation_config_key(target_dispatcher, target_vehicle),
            transportation_config
        )

    @classmethod
    def set_transportation_status(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, get_key=None):
        return kvs_client.set(
            cls.get_transportation_status_key(target_dispatcher, target_vehicle),
            transportation_status,
            get_key=get_key
        )

    @classmethod
    def set_vehicle_api_key_to_active_list(cls, kvs_client, target_dispatcher, vehicle_config, dispatcher_config):
        dispatcher_config.active_api_keys.append(vehicle_config.activation)
        dispatcher_config.inactive_api_keys.remove(vehicle_config.activation)
        return cls.set_config(kvs_client, target_dispatcher, dispatcher_config)

    @classmethod
    def set_vehicle_api_key_to_inactive_list(cls, kvs_client, target_dispatcher, vehicle_config, dispatcher_config):
        if vehicle_config.activation not in dispatcher_config.inactive_api_keys:
            dispatcher_config.inactive_api_keys.append(vehicle_config.activation)
        if vehicle_config.activation in dispatcher_config.active_api_keys:
            dispatcher_config.active_api_keys.remove(vehicle_config.activation)
        return cls.set_config(kvs_client, target_dispatcher, dispatcher_config)

    @classmethod
    def set_relation(cls, kvs_client, target_owner, relation_name, from_id, to_id):
        key = cls.get_relation_key(target_owner, relation_name, from_id, to_id)
        return kvs_client.set(key, None)

    @classmethod
    def set_route_code_lane_array_id_relation(cls, kvs_client, target, route_code, lane_array_id):
        return cls.set_relation(kvs_client, target, "route_code_lane_array_id", str(lane_array_id), route_code) * \
               cls.set_relation(kvs_client, target, "route_code_lane_array_id", route_code, str(lane_array_id))

    @classmethod
    def initialize_closest_waypoint(cls, kvs_client, target):
        lane_array = cls.get_lane_array(kvs_client, target)
        return cls.set_closest_waypoint(kvs_client, target, Autoware.Status.ClosestWaypoint.new_data(**{
            "lane_array_id": -1 if lane_array is None else lane_array.id,
            "index": 0
        }))

    @classmethod
    def initialize_state_cmd(cls, kvs_client, target):
        return cls.set_state_cmd(kvs_client, target, None)

    @classmethod
    def initialize_lane_array(cls, kvs_client, target):
        return cls.set_lane_array(kvs_client, target, None)

    @classmethod
    def initialize_vehicle_status_schedule_id(cls, kvs_client, target_vehicle):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            vehicle_schedules = cls.get_schedules(kvs_client, target_vehicle)
            if vehicle_schedules is not None:
                if vehicle_status.schedule_id is None:
                    vehicle_status.schedule_id = vehicle_schedules[0].id
                    logger.info("VEHICLE_STATUS: {}".format(vehicle_status))
                    return cls.set_status(kvs_client, target_vehicle, vehicle_status)
        return False

    @classmethod
    def update_closest_waypoint(cls, kvs_client, target, config, status):
        current_closest_waypoint = Simulator.search_closest_waypoint_from_lane_array(
            status.current_pose, status.lane_array)
        status.closest_waypoint.index = min(
            current_closest_waypoint.index + config.step_size, len(status.lane_array.lanes[0].waypoints) - 1)
        return cls.set_closest_waypoint(kvs_client, target, status.closest_waypoint)

    @classmethod
    def update_current_pose(cls, kvs_client, target, status):
        if status.lane_array is not None:
            if 0 <= status.closest_waypoint.index < len(status.lane_array.lanes[0].waypoints):
                return cls.set_current_pose(
                    kvs_client, target,
                    status.lane_array.lanes[0].waypoints[status.closest_waypoint.index].pose)
        return False

    @staticmethod
    def get_timestamp():
        nsec, sec = modf(time())
        return Autoware.Status.Header.Timestamp.new_data(
            secs=int(sec),
            nsecs=int(nsec*(10**9))
        )

    @classmethod
    def get_closest_waypoint(cls, kvs_client, target):
        key = cls.get_closest_waypoint_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.ClosestWaypoint.new_data(**value)
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
    def get_lane_array(cls, kvs_client, target):
        key = cls.get_lane_array_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.LaneArray.new_data(**value)
        return value

    @classmethod
    def get_decision_maker_state(cls, kvs_client, target):
        key = cls.get_decision_maker_state_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.DecisionMakerState.new_data(**value)
        return value

    @classmethod
    def get_light_color(cls, kvs_client, target):
        key = cls.get_light_color_key(target)
        value = kvs_client.get(key)
        if value is not None:
            value = Autoware.Status.LightColor.new_data(**value)
        return value

    @classmethod
    def get_autoware_status(cls, kvs_client, target):
        return Autoware.Status.new_data(
            current_pose=cls.get_current_pose(kvs_client, target),
            closest_waypoint=cls.get_closest_waypoint(kvs_client, target),
            state_cmd=cls.get_state_cmd(kvs_client, target),
            lane_array=cls.get_lane_array(kvs_client, target),
            decision_maker_state=cls.get_decision_maker_state(kvs_client, target),
            light_color=cls.get_light_color(kvs_client, target),
            updated_at=Schedule.get_time()
        )

    @classmethod
    def set_autoware_status(cls, kvs_client, target, value):
        set_flag = cls.set_current_pose(kvs_client, target, value.current_pose)
        set_flag *= cls.set_closest_waypoint(kvs_client, target, value.closest_waypoint)
        set_flag *= cls.set_state_cmd(kvs_client, target, value.state_cmd)
        set_flag *= cls.set_lane_array(kvs_client, target, value.lane_array)
        set_flag *= cls.set_decision_maker_state(kvs_client, target, value.decision_maker_state)
        return set_flag

    @classmethod
    def get_current_vehicle_schedule(cls, vehicle_status, vehicle_schedules):
        return Schedule.get_schedule_by_id(vehicle_schedules, vehicle_status.schedule_id)

    @classmethod
    def get_vehicle_schedule_events(cls, vehicle_schedules):
        return list(map(lambda x: x.event, vehicle_schedules))

    @classmethod
    def get_next_vehicle_schedule_index(cls, vehicle_status, vehicle_schedules):
        next_vehicle_schedule = Schedule.get_next_schedule_by_current_schedule_id(
            vehicle_schedules, vehicle_status.schedule_id)
        return vehicle_schedules.index(next_vehicle_schedule)

    @classmethod
    def get_next_vehicle_schedule_id(cls, vehicle_status, vehicle_schedules):
        return Schedule.get_next_schedule_by_current_schedule_id(vehicle_schedules, vehicle_status.schedule_id).id

    @classmethod
    def update_and_set_vehicle_status(cls, kvs_client, target, vehicle_status, new_state, vehicle_schedules=None):
        vehicle_status.state = new_state
        vehicle_status.updated_at = Schedule.get_time()
        if vehicle_schedules is not None:
            vehicle_status.schedule_id = cls.get_next_vehicle_schedule_id(vehicle_status, vehicle_schedules)
        return cls.set_status(kvs_client, target, vehicle_status)

    @classmethod
    def update_next_vehicle_schedule_end_time_with_current_time_and_duration(cls, vehicle_status, vehicle_schedules):
        current_time = Schedule.get_time()
        next_vehicle_schedule_index = cls.get_next_vehicle_schedule_index(vehicle_status, vehicle_schedules)
        duration = vehicle_schedules[next_vehicle_schedule_index].period.end - \
            vehicle_schedules[next_vehicle_schedule_index].period.start
        vehicle_schedules[next_vehicle_schedule_index].period.start = current_time
        vehicle_schedules[next_vehicle_schedule_index].period.end = current_time + duration

    @classmethod
    def generate_lane_array_from_route_code(cls, maps_client, route_code):
        return maps_client.route.get_lane_array(route_code)

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
        return maps_client.arrow.get_pose(location.arrow_code, location.waypoint_id)

    @classmethod
    def generate_state_cmd_from_data(cls, data):
        return Autoware.Status.StateCMD.new_data(data=data)

    @classmethod
    def generate_vehicle_schedules(cls, transportation_config):
        schedules = []
        start_time = Schedule.get_time()
        for event in transportation_config.events:
            schedules.append(Schedule.new_schedule(
                targets=transportation_config.targets,
                event=event.name,
                start_time=start_time,
                end_time=start_time + event.duration if "duration" in event else 0,
                route_code=event.route_code if "route_code" in event else None
            ))
            start_time += event.duration if "duration" in event else 0
        return schedules

    @classmethod
    def generate_route_point(cls, kvs_client, target, closest_waypoint):
        route_code = cls.get_route_code_from_lane_array_id(kvs_client, target, closest_waypoint.lane_array_id)
        return RoutePoint.new_data(
            route_code=route_code,
            index=closest_waypoint.index
        )

    @classmethod
    def update_vehicle_route_code(cls, kvs_client, target_vehicle):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedules = cls.get_schedules(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedules]:
            vehicle_schedule = Schedule.get_schedule_by_id(vehicle_schedules, vehicle_status.schedule_id)
            if vehicle_schedule is not None:
                if "route_code" in vehicle_schedule:
                    vehicle_status.route_point = RoutePoint.new_data(**{
                        "route_code": vehicle_schedule.route_code,
                        "index": 0
                    })
                    return cls.set_status(kvs_client, target_vehicle, vehicle_status)
        return False

    @classmethod
    def update_and_set_vehicle_pose_to_route_start(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = cls.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status.route_point is not None and \
                vehicle_status.decision_maker_state != Autoware.CONST.DECISION_MAKER_STATE.WAIT_ORDER:
            pose, location = maps_client.route.get_route_point_pose_and_location(vehicle_status.route_point)
            vehicle_status.location = location
            vehicle_status.pose = pose
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
                # on closest_waypoint_ros_message
                pose, location = maps_client.route.get_route_point_pose_and_location(vehicle_status.route_point)
                if None in [pose, location]:
                    logger.warning("Current pose is out of route.")
                else:
                    vehicle_status.location = location
                    vehicle_status.pose = pose
                    cls.set_status(kvs_client, target_vehicle, vehicle_status)

            else:
                # on current_pose_ros_message
                if vehicle_status.location is None and vehicle_status.current_pose is not None:
                    vehicle_status.location = maps_client.map_match.get_matched_location_on_arrows(
                        cls.generate_pose_from_current_pose(vehicle_status.current_pose))

                    if vehicle_status.location is not None:
                        vehicle_status.pose = cls.generate_pose_from_location(maps_client, vehicle_status.location)

                        cls.set_status(kvs_client, target_vehicle, vehicle_status)
                    else:
                        logger.warning("Current pose is out of range.")

    @classmethod
    def update_and_set_transportation_status(
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, new_state, get_key):
        transportation_status.state = new_state
        transportation_status.updated_at = Schedule.get_time()
        return cls.set_transportation_status(
            kvs_client, target_dispatcher, target_vehicle, transportation_status, get_key)

    @classmethod
    def delete_config(cls, kvs_client, target):
        key = cls.get_config_key(target)
        kvs_client.delete(key)
        return True

    @classmethod
    def delete_transportation_status(cls, kvs_client, target_dispatcher, target_vehicle):
        key = cls.get_transportation_status_key(target_dispatcher, target_vehicle)
        kvs_client.delete(key)
        return True

    @classmethod
    def delete_relation(cls, kvs_client, target_owner, relation_name, from_id):
        for key in cls.get_relation_keys(kvs_client, target_owner, relation_name, from_id):
            kvs_client.delete(key)
