#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Schedule
from ams.nodes.vehicle import StateMachine as VehicleStateMachine
from ams.nodes.autoware import CONST, Helper, Publisher


class Condition(VehicleStateMachine.EventHandler.Transition.Condition):

    VEHICLE = CONST
    Helper = Helper

    @classmethod
    def autoware_is_waiting_for_lane_waypoints_array(cls, vehicle_status):
        return vehicle_status.decision_maker_state.data in [
            cls.VEHICLE.ROS.DECISION_MAKER_STATE.WAIT_MISSION_ORDER, cls.VEHICLE.ROS.DECISION_MAKER_STATE.WAIT_ORDER
        ]

    @classmethod
    def autoware_is_waiting_for_engage_state_cmd(cls, vehicle_status):
        return vehicle_status.decision_maker_state.data == cls.VEHICLE.ROS.DECISION_MAKER_STATE.DRIVE_READY


class BeforeHook(VehicleStateMachine.EventHandler.Transition.BeforeHook):

    VEHICLE = CONST
    Helper = Helper
    Publisher = Publisher

    @classmethod
    def publish_lane_waypoint_array(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        lane_waypoint_array = cls.Helper.get_next_schedule_lane_waypoint_array(
            clients, vehicle_status, vehicle_schedules)
        cls.Publisher.publish_lane_waypoint_array(clients, target_roles, lane_waypoint_array)

    @classmethod
    def update_vehicle_route_code(cls, vehicle_status, vehicle_schedules):
        vehicle_status.route_code = cls.Helper.get_vehicle_route_code(vehicle_status, vehicle_schedules)

    @classmethod
    def publish_state_cmd_engage(cls, clients, target_roles):
        state_cmd = cls.Helper.get_state_cmd_from_data(cls.VEHICLE.ROS.STATE_CMD.ENGAGE)
        cls.Publisher.publish_state_cmd(clients, target_roles, state_cmd)


class AfterHook(VehicleStateMachine.EventHandler.Transition.AfterHook):

    Helper = Helper
    Publisher = Publisher

    # @classmethod
    # def update_traffic_signal_status_subscribers(cls):
    #     cls.Helper.update_traffic_signal_status_subscribers()


class Transition(VehicleStateMachine.EventHandler.Transition):

    VEHICLE = CONST

    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook

    @classmethod
    def start_processing_to_initialized(cls, clients, target_roles, vehicle_status, vehicle_config):
        if cls.Condition.vehicle_located(vehicle_status):
            if cls.Condition.dispatcher_assigned(vehicle_config):
                cls.Helper.update_and_set_vehicle_status(
                    clients, target_roles, vehicle_status, cls.VEHICLE.STATE.INITIALIZED)
                cls.AfterHook.publish_vehicle_config_to_target_dispatcher(clients, target_roles, vehicle_config)
                return True
        else:
            cls.Helper.update_and_set_vehicle_pose(clients, target_roles, vehicle_status)

        return False

    @classmethod
    def mission_started_to_waiting_for_decision_maker_state_drive_ready(
            cls, clients, target_roles, vehicle_status, vehicle_schedules):
        if cls.Condition.autoware_is_waiting_for_lane_waypoints_array(vehicle_status):
            cls.BeforeHook.publish_lane_waypoint_array(clients, target_roles, vehicle_status, vehicle_schedules)
            return False
        else:
            if cls.Condition.autoware_is_waiting_for_engage_state_cmd(vehicle_status):
                cls.BeforeHook.update_vehicle_route_code(vehicle_status, vehicle_schedules)
                update_flag = cls.Helper.update_and_set_vehicle_status(
                    clients, target_roles, vehicle_status, cls.VEHICLE.STATE.WAITING_FOR_AUTOWARE_STATE_DRIVE_READY,
                    vehicle_schedules)
                # if update_flag:
                #     cls.AfterHook.update_traffic_signal_status_subscribers()
                return update_flag
            else:
                return False

    @classmethod
    def waiting_for_decision_maker_state_wait_order_to_waiting_for_decision_maker_state_drive_ready(
            cls, clients, target_roles, vehicle_status, vehicle_schedules):
        if cls.Condition.autoware_is_waiting_for_lane_waypoints_array(vehicle_status):
            cls.BeforeHook.publish_lane_waypoint_array(clients, target_roles, vehicle_status, vehicle_schedules)
            return False
        else:
            if cls.Condition.autoware_is_waiting_for_engage_state_cmd(vehicle_status):
                cls.BeforeHook.update_vehicle_route_code(vehicle_status, vehicle_schedules)
                return cls.Helper.update_and_set_vehicle_status(
                    clients, target_roles, vehicle_status, cls.VEHICLE.STATE.WAITING_FOR_AUTOWARE_STATE_DRIVE_READY,
                    vehicle_schedules)
            else:
                return False

    @classmethod
    def waiting_for_decision_maker_state_drive_ready_to_waiting_for_decision_maker_state_wait_order(
            cls, clients, target_roles, vehicle_status, vehicle_schedules):
        if cls.Condition.autoware_is_waiting_for_engage_state_cmd(vehicle_status):
            cls.BeforeHook.publish_state_cmd_engage(clients, target_roles)
            return False
        else:
            # cls.BeforeHook.publish_light_color(
            #   clients, target_roles, vehicle_status, vehicle_schedules, traffic_signal_statuses)
            if cls.Condition.autoware_is_waiting_for_lane_waypoints_array(vehicle_status):
                return cls.Helper.update_and_set_vehicle_status(
                    clients, target_roles, vehicle_status, cls.VEHICLE.STATE.WAITING_FOR_AUTOWARE_STATE_WAIT_ORDER,
                    vehicle_schedules)


class EventHandler(VehicleStateMachine.EventHandler):

    VEHICLE = CONST

    Transition = Transition

    @classmethod
    def send_lane_waypoint_array(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.MISSION_STARTED:
            update_flag = cls.Transition.mission_started_to_waiting_for_decision_maker_state_drive_ready(
                clients, target_roles, vehicle_status, vehicle_schedules)
        elif state == cls.VEHICLE.STATE.WAITING_FOR_AUTOWARE_STATE_WAIT_ORDER:
            update_flag = cls.Transition.\
                waiting_for_decision_maker_state_wait_order_to_waiting_for_decision_maker_state_drive_ready(
                    clients, target_roles, vehicle_status, vehicle_schedules)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))
        return update_flag

    @classmethod
    def send_engage(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.WAITING_FOR_AUTOWARE_STATE_DRIVE_READY:
            update_flag = cls.Transition.\
                waiting_for_decision_maker_state_drive_ready_to_waiting_for_decision_maker_state_wait_order(
                    clients, target_roles, vehicle_status, vehicle_schedules)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))
        return update_flag


class StateMachine(VehicleStateMachine):

    VEHICLE = CONST

    Helper = Helper
    EventHandler = EventHandler

    @classmethod
    def update_vehicle_state(cls, clients, target_roles):
        vehicle_config_key, vehicle_config = cls.Helper.get_vehicle_config_key_and_value(clients, target_roles)
        vehicle_status_key, vehicle_status = cls.Helper.get_vehicle_status_key_and_value(clients, target_roles)
        vehicle_schedules_key, vehicle_schedules = cls.Helper.get_vehicle_schedules_key_and_value(clients, target_roles)

        state = vehicle_status.state

        if state in [
                cls.VEHICLE.STATE.START_PROCESSING, cls.VEHICLE.STATE.INITIALIZED,
                cls.VEHICLE.STATE.INACTIVE, cls.VEHICLE.STATE.END_PROCESSING
        ]:
            update_flag = cls.update_vehicle_state_without_schedules(
                clients, target_roles, vehicle_status, vehicle_config, vehicle_schedules)
        else:
            update_flag = cls.update_vehicle_state_with_schedules(
                clients, target_roles, vehicle_status, vehicle_schedules)

        return update_flag

    @classmethod
    def update_vehicle_state_without_schedules(
            cls, clients, target_roles, vehicle_status, vehicle_config, vehicle_schedules):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.START_PROCESSING:
            update_flag = cls.EventHandler.Transition.start_processing_to_initialized(
                clients, target_roles, vehicle_status, vehicle_config)
        elif state == cls.VEHICLE.STATE.INITIALIZED:
            update_flag = cls.EventHandler.Transition.initialized_to_active(
                clients, target_roles, vehicle_status, vehicle_schedules)
        elif state == cls.VEHICLE.STATE.INACTIVE:
            update_flag = cls.EventHandler.Transition.to_end_processing(clients, target_roles, vehicle_status)
        elif state == cls.VEHICLE.STATE.END_PROCESSING:
            update_flag = False
        else:
            raise ValueError("Transition from {} without event is undefined.".format(state))

        return update_flag

    @classmethod
    def update_vehicle_state_with_schedules(cls, clients, target_roles, vehicle_status, vehicle_schedules):
        update_flag = False

        if vehicle_status.schedule_id is None:
            vehicle_status.schedule_id = vehicle_schedules[0].id

        next_schedule = Schedule.get_next_schedule_by_current_schedule_id(vehicle_schedules, vehicle_status.schedule_id)

        if next_schedule is not None:
            event = next_schedule.event
            if event == cls.VEHICLE.EVENT.START_MISSION:
                update_flag = cls.EventHandler.start_mission(clients, target_roles, vehicle_status, vehicle_schedules)

            elif event == cls.VEHICLE.EVENT.SEND_LANE_WAYPOINT_ARRAY:
                update_flag = cls.EventHandler.send_lane_waypoint_array(
                    clients, target_roles, vehicle_status, vehicle_schedules)
            elif event == cls.VEHICLE.EVENT.SEND_ENGAGE:
                update_flag = cls.EventHandler.send_engage(clients, target_roles, vehicle_status, vehicle_schedules)

            elif event == cls.VEHICLE.EVENT.END_MISSION:
                update_flag = cls.EventHandler.end_mission(clients, target_roles, vehicle_status, vehicle_schedules)
            elif event == cls.VEHICLE.EVENT.DEACTIVATE:
                update_flag = cls.EventHandler.deactivate(clients, target_roles, vehicle_status, vehicle_schedules)
            else:
                raise ValueError("Unknown Event {}.".format(event))

        return update_flag
