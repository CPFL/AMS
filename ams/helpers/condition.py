#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Hook, Schedule
from ams.structures import Vehicle, Dispatcher


class Condition(object):

    @classmethod
    def received_lane_array_exists(cls, kvs_client, target_autoware):
        received_lane_array = Hook.get_received_lane_array(kvs_client, target_autoware)
        return received_lane_array is not None

    @classmethod
    def lane_array_updated(cls, kvs_client, target_autoware):
        received_lane_array = Hook.get_received_lane_array(kvs_client, target_autoware)
        lane_array = Hook.get_lane_array(kvs_client, target_autoware)
        return received_lane_array == lane_array

    @classmethod
    def lane_array_initialized(cls, kvs_client, target_autoware):
        received_lane_array = Hook.get_received_lane_array(kvs_client, target_autoware)
        lane_array = Hook.get_lane_array(kvs_client, target_autoware)
        return received_lane_array is None is lane_array

    @classmethod
    def received_lane_array_initialized(cls, kvs_client, target_autoware):
        received_lane_array = Hook.get_received_lane_array(kvs_client, target_autoware)
        return received_lane_array is None

    @classmethod
    def vehicle_location_initialized(cls, kvs_client, target_autoware):
        vehicle_location = Hook.get_vehicle_location(kvs_client, target_autoware)
        if vehicle_location is not None:
            return vehicle_location.waypoint_index == 0
        return False

    @classmethod
    def state_cmd_is_expected(cls, kvs_client, target_autoware, expected):
        state_cmd = Hook.get_state_cmd(kvs_client, target_autoware)
        return state_cmd == expected

    @classmethod
    def vehicle_location_is_end_point(cls, kvs_client, target_autoware):
        vehicle_location = Hook.get_vehicle_location(kvs_client, target_autoware)
        lane_array = Hook.get_lane_array(kvs_client, target_autoware)
        if None not in [vehicle_location, lane_array]:
            return vehicle_location.waypoint_index == len(lane_array["lanes"][0]["waypoints"]) - 1
        return False

    @classmethod
    def vehicle_located(cls, kvs_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return vehicle_status.location is not None
        return False

    @classmethod
    def dispatcher_assigned(cls, kvs_client, target_vehicle):
        vehicle_config = Hook.get_config(kvs_client, target_vehicle, Vehicle.Config)
        if vehicle_config is not None:
            return vehicle_config.target_dispatcher is not None
        return False

    @classmethod
    def vehicle_state_timeout(cls, kvs_client, target_vehicle, timeout):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return timeout < Schedule.get_time() - vehicle_status.updated_at
        return False

    @classmethod
    def vehicle_schedules_exists(cls, kvs_client, target_vehicle):
        vehicle_schedules = Hook.get_schedules(kvs_client, target_vehicle)
        if vehicle_schedules is None:
            return False
        return 0 < len(vehicle_schedules)

    @classmethod
    def vehicle_status_schedule_id_initialized(cls, kvs_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return vehicle_status.schedule_id is not None
        return False

    @classmethod
    def vehicle_route_point_updated(cls, kvs_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedules = Hook.get_schedules(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedules]:
            vehicle_schedule = Schedule.get_schedule_by_id(vehicle_schedules, vehicle_status.schedule_id)
            if vehicle_schedule.event == Dispatcher.CONST.TRANSPORTATION.EVENT.CHANGE_ROUTE:
                vehicle_schedule = Schedule.get_next_schedule_by_current_schedule_id(
                    vehicle_schedules, vehicle_status.schedule_id)
            if "route_code" in vehicle_schedule:
                return vehicle_status.route_point.route_code == vehicle_schedule.route_code
        return False

    @classmethod
    def vehicle_schedules_include_any_expected_events(cls, kvs_client, target_vehicle, expected_events):
        vehicle_schedules = Hook.get_schedules(kvs_client, target_vehicle)
        if vehicle_schedules is not None:
            schedule_events = Hook.get_vehicle_schedule_events(vehicle_schedules)
            return 0 < len(list(set(expected_events) & set(schedule_events)))
        return False

    @classmethod
    def decision_maker_state_is_expected(cls, kvs_client, target_vehicle, expected):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return "\n"+expected in vehicle_status.decision_maker_state.data
        return False

    @classmethod
    def vehicle_location_is_on_event_route(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedules = Hook.get_schedules(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedules]:
            current_schedule = Schedule.get_schedule_by_id(vehicle_schedules, vehicle_status.schedule_id)
            if current_schedule is not None:
                route_waypoint_ids = maps_client.route.get_waypoint_ids(current_schedule.route_code)
                if vehicle_status.location.waypoint_id in route_waypoint_ids:
                    return True
        return False

    @classmethod
    def vehicle_state_is_expected(cls, kvs_client, target_vehicle, expected):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return vehicle_status.state == expected
        return False

    @classmethod
    def vehicle_config_exists(cls, kvs_client, target_vehicle):
        vehicle_config = Hook.get_config(kvs_client, target_vehicle, Vehicle.Config)
        return vehicle_config is not None
