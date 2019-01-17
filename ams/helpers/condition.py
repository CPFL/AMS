#!/usr/bin/env python
# coding: utf-8

from ams import logger
from ams.helpers import Hook, Event, Target
from ams.structures import Vehicle, TrafficSignal, Dispatcher, User, CLIENT


class Condition(object):

    @classmethod
    def status_initialized(cls, kvs_client, target, structure, sub_target=None):
        return Hook.get_status(kvs_client, target, structure, sub_target) is None

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
        if vehicle_status is None:
            return None
        return vehicle_status.location is not None

    @classmethod
    def relevant_vehicle_located(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            logger.warning("dispatcher_status({}) is None".format(Target.encode(target_vehicle)))
            return None
        if dispatcher_status.vehicle_status is not None:
            return dispatcher_status.vehicle_status.location is not None
        return False

    @classmethod
    def node_state_timeout(cls, kvs_client, target, structure, timeout, sub_target=None):
        status = Hook.get_status(kvs_client, target, structure, sub_target=sub_target)
        if status is None:
            return False
        timeout_flag = timeout < Event.get_time() - status.updated_at
        if timeout_flag:
            logger.warning("Node({}) state({}) timeout".format(Target.encode(target), status.state))
        return timeout_flag

    @classmethod
    def vehicle_state_timeout(cls, kvs_client, target, timeout=5):
        return cls.node_state_timeout(kvs_client, target, Vehicle.Status, timeout)

    @classmethod
    def dispatcher_state_timeout(cls, kvs_client, target_dispatcher, target_vehicle, timeout=5):
        return cls.node_state_timeout(
            kvs_client, target_dispatcher, Dispatcher.Status, timeout, sub_target=target_vehicle)

    @classmethod
    def traffic_signal_state_timeout(cls, kvs_client, target, timeout=5):
        return cls.node_state_timeout(kvs_client, target, TrafficSignal.Status, timeout)

    @classmethod
    def user_state_timeout(cls, kvs_client, target, timeout=5):
        return cls.node_state_timeout(kvs_client, target, User.Status, timeout)

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
    def applied_schedule_initialized(cls, kvs_client, from_target, to_target):
        return Hook.get_applied_schedule(kvs_client, from_target, to_target) is None

    @classmethod
    def generated_schedule_initialized(cls, kvs_client, from_target, to_target):
        return Hook.get_generated_schedule(kvs_client, from_target, to_target) is None

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
    def received_stop_signal_initialized(cls, kvs_client, target):
        received_stop_signal = Hook.get_received_stop_signal(kvs_client, target)
        return received_stop_signal is None

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
    def vehicle_location_is_ahead_event_route(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedule]:
            current_event = Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)
            route_section = maps_client.route.generate_route_section_with_route_codes(
                current_event.route_code, vehicle_status.route_point.route_code
            )
            return vehicle_status.route_point.index < route_section.start_index
        return False

    @classmethod
    def vehicle_location_is_on_event_route(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedule]:
            current_event = Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)
            if current_event is not None:
                route_section = maps_client.route.generate_route_section_with_route_codes(
                    current_event.route_code, vehicle_status.route_point.route_code
                )
                return route_section.start_index <= vehicle_status.route_point.index <= route_section.end_index
        return False

    @classmethod
    def vehicle_location_is_behind_event_route(cls, kvs_client, maps_client, target_vehicle):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        vehicle_schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if None not in [vehicle_status, vehicle_schedule]:
            current_event = Event.get_event_by_id(vehicle_schedule.events, vehicle_status.event_id)
            route_section = maps_client.route.generate_route_section_with_route_codes(
                current_event.route_code, vehicle_status.route_point.route_code
            )
            return route_section.end_index < vehicle_status.route_point.index
        return False

    @classmethod
    def vehicle_state_is_expected(cls, kvs_client, target_vehicle, expected):
        vehicle_status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if vehicle_status is not None:
            return vehicle_status.state == expected
        return False

    @classmethod
    def relevant_vehicle_state_is_expected(cls, kvs_client, target_dispatcher, target_vehicle, expected):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            logger.warning("dispatcher_status() is None".format(Target.encode(target_vehicle)))
            return None
        if dispatcher_status.vehicle_status is None:
            logger.warning("dispatcher_status.vehicle_status() is None".format(Target.encode(target_vehicle)))
            return None
        return dispatcher_status.vehicle_status.state == expected

    @classmethod
    def vehicle_config_exists(cls, kvs_client, target_vehicle):
        vehicle_config = Hook.get_config(kvs_client, target_vehicle, Vehicle.Config)
        return vehicle_config is not None

    @classmethod
    def on_vehicle_schedule(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if None in [status, schedule]:
            return False

        event_ids = list(map(lambda x: x.id, schedule.events))
        return all([
            status.schedule_id == schedule.id,
            status.event_id in event_ids
        ])

    @classmethod
    def on_vehicle_event(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        if status is None:
            return False
        return status.on_event

    @classmethod
    def vehicle_schedule_replaceable(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        received_schedule = Hook.get_received_schedule(kvs_client, target_vehicle)
        if None in [status, received_schedule]:
            return None

        received_schedule_event_ids = list(map(lambda x: x.id, received_schedule.events))
        if status.event_id not in received_schedule_event_ids:
            logger.warning("No event id:{} in received schedule event ids:{}.".format(
                    status.event_id, received_schedule_event_ids))
            return False

        # todo: check location and time

        return True

    @classmethod
    def vehicle_schedule_applied(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, target_vehicle)
        if dispatcher_status is None:
            return False
        if dispatcher_status.vehicle_status is None:
            return False
        generated_vehicle_schedule = Hook.get_generated_schedule(kvs_client, target_dispatcher, target_vehicle)
        if generated_vehicle_schedule is None:
            return False
        return dispatcher_status.vehicle_status.schedule_id == generated_vehicle_schedule.id

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
    def applied_vehicle_schedule_replaced(cls, kvs_client, target_dispatcher, target_vehicle):
        return cls.generated_schedule_initialized(kvs_client, target_dispatcher, target_vehicle)

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

    @classmethod
    def user_hired_vehicle(cls, kvs_client, target):
        status = Hook.get_status(kvs_client, target, User.Status)
        return status.vehicle_info is not None

    @classmethod
    def vehicle_related_to_user_exists(cls, kvs_client, target_dispatcher, target_user):
        config = Hook.get_config(kvs_client, target_dispatcher, Dispatcher.Config)
        for target_vehicle in config.target_vehicles:
            status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, target_vehicle)
            if status is not None and status.user_statuses is not None:
                filtered_user_statuses = list(filter(
                    lambda x: Target.is_same(target_user, x.target),
                    status.user_statuses))
                if 1 == len(filtered_user_statuses):
                    return True
            applied_schedule = Hook.get_applied_schedule(kvs_client, target_dispatcher, target_vehicle)
            if None not in [applied_schedule, status.vehicle_status]:
                if status.vehicle_status.event_id is not None:
                    index = Event.get_event_index_by_event_id(applied_schedule.events, status.vehicle_status.event_id)
                    if any(map(
                            lambda x: Target.target_in_targets(target_user, x.targets),
                            applied_schedule.events[index:])):
                        return True
        return False

    @classmethod
    def vehicle_schedule_has_all_user_events(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, target_vehicle)
        if dispatcher_status is None:
            return None
        if None in [dispatcher_status.vehicle_status, dispatcher_status.user_statuses]:
            return None
        if dispatcher_status.vehicle_status.location is None:
            return None
        if 0 == len(dispatcher_status.user_statuses):
            return True
        applied_vehicle_schedule = Hook.get_applied_schedule(kvs_client, target_dispatcher, target_vehicle)
        if applied_vehicle_schedule is None:
            logger.info("applied_vehicle_schedule is None")
            return False
        index = Event.get_event_index_by_event_id(
            applied_vehicle_schedule.events, dispatcher_status.vehicle_status.event_id)
        user_statuses = list(filter(
            lambda us: not any(filter(
                lambda e: Target.target_in_targets(us["target"], e.targets),
                applied_vehicle_schedule.events[index:])),
            dispatcher_status.user_statuses))
        return 0 == len(user_statuses)

    @classmethod
    def vehicle_arrived_at_user_start_location(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return None
        vehicle_status = dispatcher_status.vehicle_status
        if vehicle_status.event_id is None:
            return False
        event_id_parts = vehicle_status.event_id.split(Vehicle.CONST.EVENT_ID_PARTS.DELIMITER)
        if len(event_id_parts) < 2:
            return False
        return all([
            Vehicle.CONST.EVENT_ID_PARTS.WAIT_AT_USER_START == event_id_parts[0],
            Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT == event_id_parts[1]])

    @classmethod
    def vehicle_arrived_at_user_goal_location(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return None
        vehicle_status = dispatcher_status.vehicle_status
        if vehicle_status.event_id is None:
            return False
        event_id_parts = vehicle_status.event_id.split(Vehicle.CONST.EVENT_ID_PARTS.DELIMITER)
        if len(event_id_parts) < 2:
            return False
        return all([
            Vehicle.CONST.EVENT_ID_PARTS.WAIT_AT_USER_GOAL == event_id_parts[0],
            Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT == event_id_parts[1]])

    @classmethod
    def vehicle_is_waiting_event_shift(cls, kvs_client, target_vehicle):
        status = Hook.get_status(kvs_client, target_vehicle, Vehicle.Status)
        schedule = Hook.get_schedule(kvs_client, target_vehicle)
        if None in [status, schedule]:
            return None
        if status.event_id is None:
            return False
        return Event.get_event_by_id(schedule.events, status.event_id).name == Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT

    @classmethod
    def relevant_vehicle_is_waiting_event_shift(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        applied_schedule = Hook.get_applied_schedule(kvs_client, target_dispatcher, target_vehicle)
        if None in [dispatcher_status, applied_schedule]:
            return False
        vehicle_status = dispatcher_status.vehicle_status
        if vehicle_status is None:
            return False
        if vehicle_status.event_id is None:
            return False
        event = Event.get_event_by_id(applied_schedule.events, vehicle_status.event_id)
        if event is None:
            return False
        event_name = Event.get_event_by_id(applied_schedule.events, vehicle_status.event_id).name
        return event_name == Vehicle.CONST.EVENT.WAIT_EVENT_SHIFT

    @classmethod
    def vehicle_state_in_user_status_is_expected(cls, kvs_client, target_user, expected):
        status = Hook.get_status(kvs_client, target_user, User.Status)
        if status is None:
            return None
        if status.vehicle_info is None:
            return False
        # logger.info("status.vehicle_info.state == expected: {} == {}".format(status.vehicle_info.state, expected))
        return status.vehicle_info.state == expected

    @classmethod
    def goal_location_in_user_status_is_initialized(cls, kvs_client, target_user):
        status = Hook.get_status(kvs_client, target_user, User.Status)
        if status is None:
            return None
        return status.goal_location is None

    @classmethod
    def user_status_in_transportation_finished_initialized(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return None
        event_id = dispatcher_status.vehicle_status.event_id
        target_user = Target.new_target(*event_id.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[2:4])
        return cls.status_initialized(kvs_client, target_dispatcher, User.Status, sub_target=target_user)

    @classmethod
    def transportation_finished_user_status_exists_in_user_statuses(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return None
        event_id = dispatcher_status.vehicle_status.event_id
        target_user = Target.new_target(*event_id.split(CLIENT.KVS.KEY_PATTERN_DELIMITER)[2:4])
        user_statuses = Hook.get_user_statuses(kvs_client, target_dispatcher, target_vehicle)
        return any([
            0 < len(list(filter(lambda x: Target.is_same(target_user, x["target"]), user_statuses))),
            0 < len(list(filter(lambda x: Target.is_same(target_user, x["target"]), dispatcher_status.user_statuses)))
        ])

    @classmethod
    def vehicle_schedule_id_with_dispatcher_initialized(cls, kvs_client, target_dispatcher, target_vehicle):
        dispatcher_status = Hook.get_status(kvs_client, target_dispatcher, Dispatcher.Status, sub_target=target_vehicle)
        if dispatcher_status is None:
            return None
        return dispatcher_status.vehicle_status.schedule_id is None
