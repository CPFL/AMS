#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Hook, Event
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
    def vehicle_state_timeout(cls, kvs_client, target_vehicle, timeout=5):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return timeout < Event.get_time() - vehicle_status.updated_at
        return False

    @classmethod
    def vehicle_events_exists(cls, kvs_client, target_vehicle):
        vehicle_events = Hook.get_events(kvs_client, target_vehicle)
        if vehicle_events is None:
            return False
        return 0 < len(vehicle_events)

    @classmethod
    def vehicle_received_events_exists(cls, kvs_client, target_vehicle):
        vehicle_received_events = Hook.get_received_events(kvs_client, target_vehicle)
        if vehicle_received_events is None:
            return False
        return 0 < len(vehicle_received_events)

    @classmethod
    def vehicle_events_initialized(cls, kvs_client, target_vehicle):
        return Hook.get_events(kvs_client, target_vehicle) is None

    @classmethod
    def vehicle_received_events_initialized(cls, kvs_client, target_vehicle):
        return Hook.get_received_events(kvs_client, target_vehicle) is None

    @classmethod
    def vehicle_status_event_id_initialized(cls, kvs_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_events = Hook.get_events(kvs_client, target_vehicle)
        if vehicle_status is not None:
            return vehicle_status.event_id in map(lambda x: x.id, vehicle_events)
        return False

    @classmethod
    def vehicle_route_point_updated(cls, kvs_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_events = Hook.get_events(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_events]:
            vehicle_event = Event.get_event_by_id(vehicle_events, vehicle_status.event_id)
            if vehicle_event.name == Dispatcher.CONST.TRANSPORTATION.EVENT.CHANGE_ROUTE:
                vehicle_event = Event.get_next_event_by_current_event_id(
                    vehicle_events, vehicle_status.event_id)
            if "route_code" in vehicle_event:
                return vehicle_status.route_point.route_code == vehicle_event.route_code
        return False

    @classmethod
    def vehicle_events_include_any_expected_events(cls, kvs_client, target_vehicle, expected_events):
        vehicle_events = Hook.get_events(kvs_client, target_vehicle)
        if vehicle_events is not None:
            event_names = Hook.get_vehicle_event_names(vehicle_events)
            return 0 < len(list(set(expected_events) & set(event_names)))
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
        vehicle_events = Hook.get_events(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_events]:
            current_event = Event.get_event_by_id(vehicle_events, vehicle_status.event_id)
            if current_event is not None:
                route_waypoint_ids = maps_client.route.get_waypoint_ids(current_event.route_code)
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

    @classmethod
    def vehicle_schedule_replaceable(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        received_events = Hook.get_received_events(kvs_client, target_vehicle)
        if received_events is None:
            return False

        if status.event_id not in map(lambda x: x.id, received_events):
            return False

        # todo: check location and time

        return True

    @classmethod
    def vehicle_schedule_changed(cls, kvs_client, target_vehicle):
        return cls.vehicle_received_events_initialized(kvs_client, target_vehicle)
