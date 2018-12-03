#!/usr/bin/env python
# coding: utf-8

from ams import logger
from ams.helpers import Hook, Event, Target
from ams.structures import Vehicle, TrafficSignal


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
    def node_state_timeout(cls, kvs_client, target, structure, timeout):
        status = Hook.get_status(kvs_client, target, structure)
        if status is None:
            return False
        timeout_flag = timeout < Event.get_time() - status.updated_at
        if timeout_flag:
            logger.warning("Node({}) state({}) timeout".format(Target.get_code(target), status.state))
        return timeout_flag

    @classmethod
    def vehicle_state_timeout(cls, kvs_client, target, timeout=5):
        return cls.node_state_timeout(kvs_client, target, Vehicle.Status, timeout)

    @classmethod
    def traffic_signal_state_timeout(cls, kvs_client, target, timeout=5):
        return cls.node_state_timeout(kvs_client, target, TrafficSignal.Status, timeout)

    @classmethod
    def schedule_exists(cls, kvs_client, target):
        schedule = Hook.get_schedule(kvs_client, target)
        if schedule is None:
            return False
        return 0 < len(schedule.events)

    @classmethod
    def received_schedule_exists(cls, kvs_client, target):
        received_schedule = Hook.get_received_schedule(kvs_client, target)
        if received_schedule is None:
            return False
        return 0 < len(received_schedule.events)

    @classmethod
    def traffic_signal_cycle_exists(cls, kvs_client, target):
        config = Hook.get_config(kvs_client, target, TrafficSignal.Config)
        if config is None:
            return False
        return config.cycle is not None

    @classmethod
    def vehicle_schedule_initialized(cls, kvs_client, target_vehicle):
        return Hook.get_schedule(kvs_client, target_vehicle) is None

    @classmethod
    def received_schedule_initialized(cls, kvs_client, target):
        return Hook.get_received_schedule(kvs_client, target) is None

    @classmethod
    def vehicle_received_schedule_initialized(cls, kvs_client, target_vehicle):
        return cls.received_schedule_initialized(kvs_client, target_vehicle)

    @classmethod
    def vehicle_route_point_updated(cls, kvs_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedule]:
            vehicle_event = Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)
            if "route_code" in vehicle_event and vehicle_status.route_point is not None:
                return vehicle_status.route_point.route_code == vehicle_event.route_code
        return False

    @classmethod
    def decision_maker_state_is_expected(cls, kvs_client, target_vehicle, expected):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return "\n"+expected in vehicle_status.decision_maker_state.data
        return False

    @classmethod
    def decision_maker_state_is_in_expected_states(cls, kvs_client, target_vehicle, expected_states):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return any(map(lambda x: "\n"+x in vehicle_status.decision_maker_state.data, expected_states))
        return False

    @classmethod
    def vehicle_location_is_on_event_route(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedule]:
            current_event = Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)
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
    def on_vehicle_schedule(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if schedule is None:
            return False

        event_ids = list(map(lambda x: x.id, schedule.events))
        return all([
            status.schedule_id == schedule.id,
            status.event_id in event_ids
        ])

    @classmethod
    def on_vehicle_event(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        return status.on_event

    @classmethod
    def vehicle_schedule_replaceable(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        received_schedule = Hook.get_received_schedule(kvs_client, target_vehicle)
        if received_schedule is None:
            return False

        received_schedule_event_ids = list(map(lambda x: x.id, received_schedule.events))
        if status.event_id not in received_schedule_event_ids:
            logger.warning("No event id:{} in received schedule event ids:{}.".format(
                    status.event_id, received_schedule_event_ids))
            return False

        # todo: check location and time

        return True

    @classmethod
    def traffic_signal_schedule_replaceable(cls, kvs_client, target_traffic_signal):
        status = Hook.get_status(kvs_client, target_traffic_signal, TrafficSignal.Status)
        received_schedule = Hook.get_received_schedule(kvs_client, target_traffic_signal)
        if received_schedule is None:
            return False

        if status.event_id not in map(lambda x: x.id, received_schedule.events):
            return False

        # todo: check color and time

        return True

    @classmethod
    def event_close_to_the_end(cls, kvs_client, target, margin=5.0):
        schedule = Hook.get_schedule(kvs_client, target)
        if schedule is None:
            return False
        event = Event.get_event_by_specified_time(schedule.events, Event.get_time())
        if event is None:
            return False
        return event.period.end < Event.get_time() + margin

    @classmethod
    def schedule_close_to_the_end(cls, kvs_client, target, margin=5.0):
        schedule = Hook.get_schedule(kvs_client, target)
        if schedule is None:
            return True
        current_time = Event.get_time()
        if len(list(filter(lambda x: current_time < x.period.end, schedule.events))) < 3:
            return True
        return schedule.events[-1].period.end < current_time + margin

    @classmethod
    def schedule_replaced(cls, kvs_client, target):
        return cls.received_schedule_initialized(kvs_client, target)

    @classmethod
    def vehicle_schedule_changed(cls, kvs_client, target_vehicle):
        return cls.schedule_replaced(kvs_client, target_vehicle)

    @classmethod
    def traffic_signal_light_color_updated(cls, kvs_client, target):
        status = Hook.get_status(kvs_client, target, TrafficSignal.Status)
        schedule = Hook.get_schedule(kvs_client, target)
        event = Event.get_event_by_specified_time(schedule.events, Event.get_time())
        return status.light_color == event.name and status.next_light_color is None is status.next_update_time

    @classmethod
    def traffic_signal_next_light_color_updated(cls, kvs_client, target):
        status = Hook.get_status(kvs_client, target, TrafficSignal.Status)
        return None not in [status.next_light_color, status.next_update_time]
