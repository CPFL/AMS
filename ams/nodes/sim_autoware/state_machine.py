#!/usr/bin/env python
# coding: utf-8

from ams.nodes.sim_autoware import CONST, Structure, Helper


class Condition(object):

    AUTOWARE = CONST

    @classmethod
    def lane_waypoints_array_exists(cls, status):
        return status.lane_waypoints_array is not None

    @classmethod
    def closest_waypoint_initialized(cls, status):
        if status.closest_waypoint is None:
            return False
        return status.closest_waypoint.data == 0

    @classmethod
    def state_cmd_is_engage(cls, status):
        if status.state_cmd is None:
            return False
        return status.state_cmd.data == cls.AUTOWARE.STATE_CMD.ENGAGE

    @classmethod
    def state_cmd_initialized(cls, status):
        return status.state_cmd is None

    @classmethod
    def closest_waypoint_is_end_point(cls, status):
        return status.closest_waypoint.data == len(status.lane_waypoints_array.lanes[0].waypoints) - 1


class BeforeHook(object):

    AUTOWARE = CONST
    Helper = Helper

    @classmethod
    def initialize_closest_waypoint(cls, clients, target_roles):
        cls.Helper.initialize_closest_waypoint(clients, target_roles)

    @classmethod
    def initialize_state_cmd(cls, clients, target_roles):
        cls.Helper.initialize_state_cmd(clients, target_roles)

    @classmethod
    def initialize_lane_waypoints_array(cls, clients, target_roles):
        cls.Helper.initialize_lane_waypoints_array(clients, target_roles)

    @classmethod
    def update_closest_waypoint(cls, clients, target_roles, config, status):
        cls.Helper.update_closest_waypoint(clients, target_roles, config, status)

    @classmethod
    def update_current_pose(cls, clients, target_roles, status):
        cls.Helper.update_current_pose(clients, target_roles, status)


class Transition(object):

    AUTOWARE = CONST
    Structure = Structure
    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook

    @classmethod
    def wait_mission_order_to_mission_check(cls, clients, target_roles, status):
        if cls.Condition.lane_waypoints_array_exists(status):
            return cls.Helper.set_decision_maker_state(
                clients, target_roles,
                cls.Structure.Status.DecisionMakerState.new_data(data=cls.AUTOWARE.DECISION_MAKER_STATE.MISSION_CHECK))
        return False

    @classmethod
    def wait_order_to_mission_check(cls, clients, target_roles, status):
        if cls.Condition.lane_waypoints_array_exists(status):
            return cls.Helper.set_decision_maker_state(
                clients, target_roles,
                cls.Structure.Status.DecisionMakerState.new_data(data=cls.AUTOWARE.DECISION_MAKER_STATE.MISSION_CHECK))
        return False

    @classmethod
    def mission_check_to_drive_ready(cls, clients, target_roles, status):
        if not cls.Condition.closest_waypoint_initialized(status):
            cls.BeforeHook.initialize_closest_waypoint(clients, target_roles)
        else:
            cls.BeforeHook.update_current_pose(clients, target_roles, status)
            return cls.Helper.set_decision_maker_state(
                clients, target_roles,
                cls.Structure.Status.DecisionMakerState.new_data(data=cls.AUTOWARE.DECISION_MAKER_STATE.DRIVE_READY))
        return False

    @classmethod
    def drive_ready_to_drive(cls, clients, target_roles, status):
        if cls.Condition.state_cmd_is_engage(status):
            return cls.Helper.set_decision_maker_state(
                clients, target_roles,
                cls.Structure.Status.DecisionMakerState.new_data(data=cls.AUTOWARE.DECISION_MAKER_STATE.DRIVE))
        return False

    @classmethod
    def drive_to_wait_order(cls, clients, target_roles, config, status):
        if not cls.Condition.state_cmd_initialized(status):
            cls.BeforeHook.initialize_state_cmd(clients, target_roles)
        else:
            if cls.Condition.lane_waypoints_array_exists(status):
                if not cls.Condition.closest_waypoint_is_end_point(status):
                    cls.BeforeHook.update_closest_waypoint(clients, target_roles, config, status)
                    cls.BeforeHook.update_current_pose(clients, target_roles, status)
                else:
                    cls.BeforeHook.initialize_lane_waypoints_array(clients, target_roles)
            else:
                return cls.Helper.set_decision_maker_state(
                    clients, target_roles,
                    cls.Structure.Status.DecisionMakerState.new_data(data=cls.AUTOWARE.DECISION_MAKER_STATE.WAIT_ORDER))
        return False


class EventHandler(object):

    AUTOWARE = CONST
    Transition = Transition


class StateMachine(object):

    AUTOWARE = CONST
    Helper = Helper
    EventHandler = EventHandler

    @classmethod
    def update(cls, clients, target_roles):
        config = cls.Helper.get_config(clients, target_roles)
        status = cls.Helper.get_status(clients, target_roles)

        state = status.decision_maker_state.data

        if state == cls.AUTOWARE.DECISION_MAKER_STATE.WAIT_MISSION_ORDER:
            update_flag = cls.EventHandler.Transition.wait_mission_order_to_mission_check(
                clients, target_roles, status)
        elif state == cls.AUTOWARE.DECISION_MAKER_STATE.WAIT_ORDER:
            update_flag = cls.EventHandler.Transition.wait_order_to_mission_check(
                clients, target_roles, status)
        elif state == cls.AUTOWARE.DECISION_MAKER_STATE.MISSION_CHECK:
            update_flag = cls.EventHandler.Transition.mission_check_to_drive_ready(
                clients, target_roles, status)
        elif state == cls.AUTOWARE.DECISION_MAKER_STATE.DRIVE_READY:
            update_flag = cls.EventHandler.Transition.drive_ready_to_drive(
                clients, target_roles, status)
        elif state == cls.AUTOWARE.DECISION_MAKER_STATE.DRIVE:
            update_flag = cls.EventHandler.Transition.drive_to_wait_order(
                clients, target_roles, config, status)
        else:
            raise ValueError("Transition from {} without event is undefined.".format(state))

        return update_flag
