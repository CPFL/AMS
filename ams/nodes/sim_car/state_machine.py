#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Schedule
from ams.nodes.vehicle import StateMachine as VehicleStateMachine
from ams.nodes.sim_car import CONST, Helper, Publisher


class Condition(VehicleStateMachine.EventHandler.Transition.Condition):

    VEHICLE = CONST
    Helper = Helper

    @classmethod
    def achieved_route_goal(cls, vehicle_status, vehicle_schedules):
        current_vehicle_schedule = cls.Helper.get_current_vehicle_schedule(vehicle_status, vehicle_schedules)
        print("achieved_route_goal {} / {} | {}".format(
            vehicle_status.location.waypoint_id, current_vehicle_schedule.route.arrow_codes,
            vehicle_status.schedule_id))
        return vehicle_status.location.waypoint_id == current_vehicle_schedule.route.goal_waypoint_id

    @classmethod
    def current_vehicle_schedule_timeout(cls, vehicle_status, vehicle_schedules):
        return cls.Helper.current_vehicle_schedule_timeout(vehicle_status, vehicle_schedules)


class BeforeHook(VehicleStateMachine.EventHandler.Transition.BeforeHook):

    VEHICLE = CONST
    Helper = Helper
    Publisher = Publisher

    @classmethod
    def update_vehicle_location_and_pose_match_to_route_start(cls, vehicle_status, vehicle_schedules, maps_client):
        print("update_vehicle_location_and_pose_match_to_route_start")
        vehicle_status.location, vehicle_status.pose = cls.Helper.get_next_vehicle_schedule_start_location_and_pose(
            vehicle_status, vehicle_schedules, maps_client)

    @classmethod
    def change_vehicle_velocity(cls, vehicle_status, new_velocity):
        vehicle_status.velocity = new_velocity


class AfterHook(VehicleStateMachine.EventHandler.Transition.AfterHook):

    Helper = Helper
    Publisher = Publisher


class Transition(VehicleStateMachine.EventHandler.Transition):

    VEHICLE = CONST

    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook

    @classmethod
    def mission_started_to_stop(cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules):
        return cls.Helper.update_and_set_vehicle_status(
            kvs_client, target_vehicle, vehicle_status, cls.VEHICLE.STATE.STOP, vehicle_schedules)

    @classmethod
    def mission_started_to_move(cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client):
        cls.BeforeHook.update_vehicle_location_and_pose_match_to_route_start(
            vehicle_status, vehicle_schedules, maps_client)
        return cls.Helper.update_and_set_vehicle_status(
            kvs_client, target_vehicle, vehicle_status, cls.VEHICLE.STATE.MOVE, vehicle_schedules)

    @classmethod
    def move_to_stop(cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules):
        if cls.Condition.achieved_route_goal(vehicle_status, vehicle_schedules):
            cls.BeforeHook.change_vehicle_velocity(vehicle_status, 0.0)
            return cls.Helper.update_and_set_vehicle_status(
                kvs_client, target_vehicle, vehicle_status, cls.VEHICLE.STATE.STOP, vehicle_schedules)
        return False

    @classmethod
    def move_to_move(cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client):
        if cls.Condition.achieved_route_goal(vehicle_status, vehicle_schedules):
            cls.BeforeHook.update_vehicle_location_and_pose_match_to_route_start(
                vehicle_status, vehicle_schedules, maps_client)
            return cls.Helper.update_and_set_vehicle_status(
                kvs_client, target_vehicle, vehicle_status, cls.VEHICLE.STATE.MOVE, vehicle_schedules)
        return False

    @classmethod
    def stop_to_move(cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client):
        if cls.Condition.current_vehicle_schedule_timeout(vehicle_status, vehicle_schedules):
            cls.BeforeHook.update_vehicle_location_and_pose_match_to_route_start(
                vehicle_status, vehicle_schedules, maps_client)
            return cls.Helper.update_and_set_vehicle_status(
                kvs_client, target_vehicle, vehicle_status, cls.VEHICLE.STATE.MOVE, vehicle_schedules)
        return False


class EventHandler(VehicleStateMachine.EventHandler):

    VEHICLE = CONST

    Transition = Transition

    @classmethod
    def move(cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.MISSION_STARTED:
            update_flag = cls.Transition.mission_started_to_move(
                kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client)
        elif state == cls.VEHICLE.STATE.STOP:
            update_flag = cls.Transition.stop_to_move(
                kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client)
        elif state == cls.VEHICLE.STATE.MOVE:
            update_flag = cls.Transition.move_to_move(
                kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))
        return update_flag

    @classmethod
    def stop(cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules):
        state = vehicle_status.state
        if state == cls.VEHICLE.STATE.MISSION_STARTED:
            update_flag = cls.Transition.mission_started_to_stop(
                kvs_client, target_vehicle, vehicle_status, vehicle_schedules)
        elif state == cls.VEHICLE.STATE.STOP:
            update_flag = cls.Transition.stop_to_stop(kvs_client, target_vehicle, vehicle_status, vehicle_schedules)
        elif state == cls.VEHICLE.STATE.MOVE:
            update_flag = cls.Transition.move_to_stop(kvs_client, target_vehicle, vehicle_status, vehicle_schedules)
        else:
            raise ValueError("Transition from {} is undefined.".format(state))
        return update_flag


class StateMachine(VehicleStateMachine):

    VEHICLE = CONST

    Helper = Helper
    EventHandler = EventHandler

    @classmethod
    def update_vehicle_state_with_schedules(
            cls, kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client):
        update_flag = False

        if vehicle_status.schedule_id is None:
            vehicle_status.schedule_id = vehicle_schedules[0].id

        next_schedule = Schedule.get_next_schedule_by_current_schedule_id(vehicle_schedules, vehicle_status.schedule_id)

        if next_schedule is not None:
            event = next_schedule.event
            if event == cls.VEHICLE.EVENT.START_MISSION:
                update_flag = cls.EventHandler.start_mission(
                    kvs_client, target_vehicle, vehicle_status, vehicle_schedules)
            elif event == cls.VEHICLE.EVENT.MOVE:
                update_flag = cls.EventHandler.move(
                    kvs_client, target_vehicle, vehicle_status, vehicle_schedules, maps_client)
            elif event == cls.VEHICLE.EVENT.STOP:
                update_flag = cls.EventHandler.stop(
                    kvs_client, target_vehicle, vehicle_status, vehicle_schedules)
            elif event == cls.VEHICLE.EVENT.END_MISSION:
                update_flag = cls.EventHandler.end_mission(
                    kvs_client, target_vehicle, vehicle_status, vehicle_schedules)
            elif event == cls.VEHICLE.EVENT.DEACTIVATE:
                update_flag = cls.EventHandler.deactivate(
                    kvs_client, target_vehicle, vehicle_status, vehicle_schedules)
            else:
                raise ValueError("Unknown Event {}.".format(event))

        return update_flag
