#!/usr/bin/env python
# coding: utf-8

from time import time
from copy import deepcopy
from pprint import pformat

from transitions import Machine

from ams import logger, Topic, Route, Schedule, Target
from ams.nodes import FleetManager
from ams.messages import UserStatus, VehicleStatus
from ams.structures import FLEET_MANAGER, SIM_TAXI_FLEET, USER, SIM_TAXI_USER, VEHICLE, SIM_TAXI


class SimTaxiFleet(FleetManager):

    CONST = SIM_TAXI_FLEET

    def __init__(self, _id, name, waypoint, arrow, route):
        super().__init__(_id, name, waypoint, arrow, route)

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route

        self.user_statuses = self.manager.dict()
        self.user_statuses_lock = self.manager.Lock()

        self.vehicle_statuses = self.manager.dict()
        self.vehicle_statuses_lock = self.manager.Lock()

        self.user_schedules = {}
        self.vehicle_schedules = {}
        self.state_machines = {}

        self.__topicPubUserSchedules = Topic()
        self.__topicPubUserSchedules.set_categories(FLEET_MANAGER.TOPIC.CATEGORIES.SCHEDULES)

        self.__topicPubVehicleSchedules = Topic()
        self.__topicPubVehicleSchedules.set_categories(FLEET_MANAGER.TOPIC.CATEGORIES.SCHEDULES)

        self.__topicSubUserStatus = Topic()
        self.__topicSubUserStatus.set_targets(Target.new_target(None, SIM_TAXI_USER.NODE_NAME), None)
        self.__topicSubUserStatus.set_categories(USER.TOPIC.CATEGORIES.STATUS)
        self.__topicSubUserStatus.set_message(UserStatus)
        self.set_subscriber(self.__topicSubUserStatus, self.update_user_status)

        self.__topicSubVehicleStatus = Topic()
        self.__topicSubVehicleStatus.set_targets(Target.new_target(None, SIM_TAXI.NODE_NAME), None)
        self.__topicSubVehicleStatus.set_categories(VEHICLE.TOPIC.CATEGORIES.STATUS)
        self.__topicSubVehicleStatus.set_message(VehicleStatus)
        self.set_subscriber(self.__topicSubVehicleStatus, self.update_vehicle_status)

    def __publish_user_schedules(self, user_id, payload):
        self.__topicPubUserSchedules.set_targets(self.target, Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME))
        self.publish(self.__topicPubUserSchedules, payload)

    def __publish_vehicle_schedules(self, vehicle_id, payload):
        self.__topicPubVehicleSchedules.set_targets(self.target, Target.new_target(vehicle_id, SIM_TAXI.NODE_NAME))
        self.publish(self.__topicPubVehicleSchedules, payload)

    def update_user_status(self, _client, _userdata, topic, payload):
        user_id = self.__topicSubUserStatus.get_from_id(topic)
        user_status = self.__topicSubUserStatus.unserialize(payload)

        self.user_statuses_lock.acquire()
        if user_id in self.user_statuses or user_status.state == USER.STATE.LOG_IN:
            self.user_statuses[user_id] = user_status
        self.user_statuses_lock.release()

    def update_vehicle_status(self, _client, _userdata, topic, payload):
        vehicle_id = self.__topicSubVehicleStatus.get_from_id(topic)
        vehicle_status = self.__topicSubVehicleStatus.unserialize(payload)

        self.vehicle_statuses_lock.acquire()
        self.vehicle_statuses[vehicle_id] = vehicle_status
        self.vehicle_statuses_lock.release()

    def update_user_schedules(self, user_statuses):
        for user_id, user_status in user_statuses.items():
            if user_id not in self.user_schedules:
                self.user_schedules[user_id] = [user_status.schedule]
            else:
                while self.user_schedules[user_id][0].id != user_status.schedule.id:
                    self.user_schedules[user_id].pop(0)
                dif_time = user_status.schedule.period.start - self.user_schedules[user_id][0].period.start
                self.user_schedules[user_id] = \
                    Schedule.get_shifted_schedules(self.user_schedules[user_id], dif_time)

    def update_vehicle_schedules(self, vehicle_statuses):
        for vehicle_id, vehicle_status in vehicle_statuses.items():
            if vehicle_id not in self.vehicle_schedules:
                self.vehicle_schedules[vehicle_id] = [vehicle_status.schedule]
            else:
                while self.vehicle_schedules[vehicle_id][0].id != vehicle_status.schedule.id:
                    self.vehicle_schedules[vehicle_id].pop(0)
                dif_time = vehicle_status.schedule.period.start - self.vehicle_schedules[vehicle_id][0].period.start
                self.vehicle_schedules[vehicle_id] = \
                    Schedule.get_shifted_schedules(self.vehicle_schedules[vehicle_id], dif_time)

    def get_dispatchable_vehicle_ids(self, user_status):
        dispatchable_vehicle_ids = list(filter(
            lambda x:
                self.waypoint.get_geohash(
                    self.vehicle_schedules[x][-1].route.goal_waypoint_id
                )[:SIM_TAXI_FLEET.DISPATCHABLE_GEOHASH_DIGIT] ==
                self.waypoint.get_geohash(
                    user_status.trip_schedules[0].route.start_waypoint_id
                )[:SIM_TAXI_FLEET.DISPATCHABLE_GEOHASH_DIGIT],
            self.vehicle_schedules.keys()
        ))
        return dispatchable_vehicle_ids

    def get_user_request_schedules(self, user_id, current_time):
        user_request_schedule = Schedule.new_schedule(
            [
                Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME)
            ],
            SIM_TAXI_USER.TRIGGER.REQUEST, current_time, current_time + 1,
        )
        user_schedules = Schedule.get_merged_schedules(
            self.user_schedules[user_id], [user_request_schedule]
        )
        return user_schedules

    def get_taxi_schedules(self, user_id, user_status, current_time):
        pickup_route, vehicle_id = self.get_pickup_route(user_status)
        if pickup_route is None:
            return None, None

        carry_route = self.get_carry_route(user_status)
        if carry_route is None:
            return None, None

        if self.vehicle_schedules[vehicle_id][-1].event == SIM_TAXI.STATE.STAND_BY:
            current_time = self.vehicle_schedules[vehicle_id][-1].period.start

        pickup_schedules, get_on_schedules = self.get_pickup_schedules(vehicle_id, user_id, pickup_route, current_time)

        carry_schedules, get_out_schedules = \
            self.get_carry_schedules(vehicle_id, user_id, carry_route, pickup_schedules[-1].period.end)

        deploy_schedules = self.get_deploy_schedules(vehicle_id, carry_route, carry_schedules[-1].period.end)

        vehicle_schedules = Schedule.get_merged_schedules(
            self.vehicle_schedules[vehicle_id], pickup_schedules + carry_schedules + deploy_schedules
        )

        user_schedules = Schedule.get_merged_schedules(
            self.user_schedules[user_id], get_on_schedules + get_out_schedules
        )
        return vehicle_id, vehicle_schedules, user_schedules

    def get_pickup_route(self, user_status):
        start_point = {
            "arrow_code": user_status.trip_schedules[0].route.arrow_codes[0],
            "waypoint_id": user_status.trip_schedules[0].route.start_waypoint_id,
        }
        vehicle_ids = self.get_dispatchable_vehicle_ids(user_status)
        if len(vehicle_ids) == 0:
            logger.warning("no dispatchable vehicles")
            return None, None

        goal_points = []
        for vehicle_id, goal_waypoint_id, goal_arrow_code in map(
                lambda x: (
                        x,
                        self.vehicle_schedules[x][-1].route.goal_waypoint_id,
                        self.vehicle_schedules[x][-1].route.arrow_codes[-1]),
                vehicle_ids):
            goal_points.append({
                "goal_id": vehicle_id,
                "arrow_code": goal_arrow_code,
                "waypoint_id": goal_waypoint_id,
            })
        routes = self.route.get_shortest_routes(start_point, goal_points, reverse=True)
        if len(routes) == 0:
            logger.warning("no pickup_route")
            return None, None
        pickup_route = min(routes.items(), key=lambda x: x[1]["cost"] + self.vehicle_schedules[x[0]][-1].period.end)[1]
        vehicle_id = pickup_route.pop("goal_id")
        pickup_route.pop("cost")
        return pickup_route, vehicle_id

    def get_carry_route(self, user_status):
        start_point = {
            "arrow_code": user_status.trip_schedules[0].route.arrow_codes[0],
            "waypoint_id": user_status.trip_schedules[0].route.start_waypoint_id,
        }
        goal_points = [{
            "goal_id": None,
            "arrow_code": user_status.trip_schedules[0].route.arrow_codes[-1],
            "waypoint_id": user_status.trip_schedules[0].route.goal_waypoint_id,
        }]
        routes = self.route.get_shortest_routes(start_point, goal_points, reverse=False)
        if len(routes) == 0:
            logger.warning("cant carry_route")
            return None
        carry_route = routes[None]
        carry_route.pop("cost")
        carry_route.pop("goal_id")
        return carry_route

    def get_deploy_route(self):
        pass

    @staticmethod
    def get_pickup_schedules(vehicle_id, user_id, pickup_route, current_time):
        targets = [
            Target.new_target(vehicle_id, SIM_TAXI.NODE_NAME),
            Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME)]
        move_for_picking_up_schedule = Schedule.new_schedule(
            targets, SIM_TAXI.TRIGGER.MOVE, current_time, current_time + 1000, pickup_route)
        stop_for_picking_up_schedule = Schedule.new_schedule(
            targets, SIM_TAXI.TRIGGER.STOP, current_time + 1000, current_time + 1010,
            Route.new_point_route(pickup_route.goal_waypoint_id, pickup_route.arrow_codes[-1]))

        waiting_schedule = Schedule.new_schedule(
            targets, SIM_TAXI_USER.TRIGGER.WAIT, current_time, current_time + 1000,
            Route.new_point_route(pickup_route.goal_waypoint_id, pickup_route.arrow_codes[-1]))
        getting_on_schedule = Schedule.new_schedule(
            targets, SIM_TAXI_USER.TRIGGER.GET_ON, current_time + 1000, current_time + 1010,
            Route.new_point_route(pickup_route.goal_waypoint_id, pickup_route.arrow_codes[-1]))
        got_on_schedule = Schedule.new_schedule(
            targets, SIM_TAXI_USER.TRIGGER.GOT_ON, current_time + 1010, current_time + 1011,
            Route.new_point_route(pickup_route.goal_waypoint_id, pickup_route.arrow_codes[-1]))
        return [move_for_picking_up_schedule, stop_for_picking_up_schedule],\
               [waiting_schedule, getting_on_schedule, got_on_schedule]

    @staticmethod
    def get_carry_schedules(vehicle_id, user_id, carry_route, current_time):
        targets = [
            Target.new_target(vehicle_id, SIM_TAXI.NODE_NAME),
            Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME)]
        move_for_discharging_schedules = Schedule.new_schedule(
            targets, SIM_TAXI.TRIGGER.MOVE, current_time, current_time+1010, carry_route)
        stop_for_discharging_schedules = Schedule.new_schedule(
            targets, SIM_TAXI.TRIGGER.STOP, current_time+1010, current_time+1020,
            Route.new_point_route(carry_route.goal_waypoint_id, carry_route.arrow_codes[-1]))

        move_schedule = Schedule.new_schedule(
            targets, SIM_TAXI_USER.TRIGGER.MOVE_VEHICLE, current_time, current_time + 1010, carry_route)
        getting_out_schedule = Schedule.new_schedule(
            targets, SIM_TAXI_USER.TRIGGER.GET_OUT, current_time + 1010, current_time + 1020,
            Route.new_point_route(carry_route.goal_waypoint_id, carry_route.arrow_codes[-1]))
        got_out_schedule = Schedule.new_schedule(
            [Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME)],
            SIM_TAXI_USER.TRIGGER.GOT_OUT, current_time + 1020, current_time + 1021,
            Route.new_point_route(carry_route.goal_waypoint_id, carry_route.arrow_codes[-1]))
        log_out_schedule = Schedule.new_schedule(
            [Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME)],
            USER.TRIGGER.LOG_OUT, current_time + 1021, current_time + 1022,
            Route.new_point_route(carry_route.goal_waypoint_id, carry_route.arrow_codes[-1]))
        return [move_for_discharging_schedules, stop_for_discharging_schedules],\
               [move_schedule, getting_out_schedule, got_out_schedule, log_out_schedule]

    @staticmethod
    def get_deploy_schedules(vehicle_id, carry_route, current_time):
        stand_by_schedules = Schedule.new_schedule(
            [
                Target.new_target(vehicle_id, SIM_TAXI.NODE_NAME),
            ],
            SIM_TAXI.TRIGGER.STAND_BY, current_time, current_time+86400,
            Route.new_point_route(carry_route.goal_waypoint_id, carry_route.arrow_codes[-1])
        )
        return [stand_by_schedules]

    def get_state_machine(self, initial_state):
        machine = Machine(
            states=list(SIM_TAXI_FLEET.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": SIM_TAXI_FLEET.TRIGGER.WAIT_USER_REQUEST,
                "source": SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_LOG_IN,
                "dest": SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_REQUEST,
                "conditions": [self.condition_user_state_and_publish_user_request_schedules]
            },
            {
                "trigger": SIM_TAXI_FLEET.TRIGGER.DISPATCH,
                "source": SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_REQUEST,
                "dest": SIM_TAXI_FLEET.STATE.WAITING_FOR_TAXI_ARRIVE_AT_USER_LOCATION,
                "conditions": [self.condition_user_state_and_publish_new_taxi_schedules]
            },
            {
                "trigger": SIM_TAXI_FLEET.TRIGGER.NOTICE,
                "source": SIM_TAXI_FLEET.STATE.WAITING_FOR_TAXI_ARRIVE_AT_USER_LOCATION,
                "dest": SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_GETTING_ON,
                "conditions": [self.condition_user_vehicle_state_and_publish_user_schedules]
            },
            {
                "trigger": SIM_TAXI_FLEET.TRIGGER.WAIT_TAXI_ARRIVAL,
                "source": SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_GETTING_ON,
                "dest": SIM_TAXI_FLEET.STATE.WAITING_FOR_TAXI_ARRIVE_AT_USER_DESTINATION,
                "conditions": [self.condition_user_state_and_publish_updated_taxi_schedules]
            },
            {
                "trigger": SIM_TAXI_FLEET.TRIGGER.NOTICE,
                "source": SIM_TAXI_FLEET.STATE.WAITING_FOR_TAXI_ARRIVE_AT_USER_DESTINATION,
                "dest": SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_GETTING_OUT,
                "conditions": [self.condition_user_vehicle_state_and_publish_user_schedules]
            },

        ])
        return machine

    def after_state_change_publish_user_schedules(self, user_id):
        self.__publish_user_schedules(
            user_id, self.__topicPubUserSchedules.serialize(self.user_schedules[user_id]))
        return True

    def after_state_change_publish_vehicle_schedules(self, vehicle_id):
        self.__publish_vehicle_schedules(
            vehicle_id, self.__topicPubVehicleSchedules.serialize(self.vehicle_schedules[vehicle_id]))
        return True

    def after_state_change_add_relation(self, user_id, vehicle_id):
        self.relation.add_relation(
            Target.new_target(vehicle_id, SIM_TAXI.NODE_NAME),
            Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME))
        return True

    @staticmethod
    def condition_user_state(user_status, expected_state):
        return user_status.state == expected_state

    def condition_user_state_and_publish_user_request_schedules(
            self, user_id, user_status, excepted_state, current_time):
        if self.condition_user_state(user_status, excepted_state):
            self.user_schedules[user_id] = self.get_user_request_schedules(user_id, current_time)
            self.after_state_change_publish_user_schedules(user_id)
            return True
        return False

    def condition_user_state_and_publish_new_taxi_schedules(
            self, user_id, user_status, excepted_state, current_time):
        if self.condition_user_state(user_status, excepted_state):
            vehicle_id, vehicle_schedules, user_schedules = self.get_taxi_schedules(user_id, user_status, current_time)
            if vehicle_id is not None:
                self.vehicle_schedules[vehicle_id] = vehicle_schedules
                self.user_schedules[user_id] = user_schedules
                self.after_state_change_publish_vehicle_schedules(vehicle_id)
                self.after_state_change_publish_user_schedules(user_id)
                self.after_state_change_add_relation(user_id, vehicle_id)
                return True
        return False

    def condition_user_state_and_publish_updated_taxi_schedules(
            self, user_id, user_status, excepted_state, vehicle_id, current_time):
        if self.condition_user_state(user_status, excepted_state):
            self.user_schedules[user_id][0].period.end = current_time
            self.vehicle_schedules[vehicle_id][0].period.end = current_time
            self.after_state_change_publish_vehicle_schedules(vehicle_id)
            self.after_state_change_publish_user_schedules(user_id)
            return True
        return False

    @staticmethod
    def condition_user_vehicle_state(user_status, vehicle_status, expected_states):
        return [user_status.state, vehicle_status.state] == expected_states

    def condition_user_vehicle_state_and_publish_user_schedules(
            self, user_id, user_status, vehicle_status, expected_states, current_time):
        if self.condition_user_vehicle_state(user_status, vehicle_status, expected_states):
            self.user_schedules[user_id][0].period.end = current_time
            self.after_state_change_publish_user_schedules(user_id)
            return True
        return False

    def get_user_statuses_and_lock(self):
        self.user_statuses_lock.acquire()
        return deepcopy(self.user_statuses)

    def set_user_statuses_and_unlock(self, user_statuses):
        self.user_statuses.clear()
        self.user_statuses.update(user_statuses)
        self.user_statuses_lock.release()

    def get_vehicle_statuses_and_lock(self):
        self.vehicle_statuses_lock.acquire()
        return deepcopy(self.vehicle_statuses)

    def set_vehicle_statuses_and_unlock(self, vehicle_statuses):
        self.vehicle_statuses.clear()
        self.vehicle_statuses.update(vehicle_statuses)
        self.vehicle_statuses_lock.release()

    def cleanup_status(self, user_statuses):
        user_ids = []
        for user_id, user_status in user_statuses.items():
            if SIM_TAXI_FLEET.TIMEOUT < time() - user_status.time:
                user_ids.append(user_id)
        for user_id in user_ids:
            self.relation.remove_relations_of(Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME))
            user_statuses.pop(user_id)
            self.user_schedules.pop(user_id)

    def update_status(self):
        user_statuses = self.get_user_statuses_and_lock()
        vehicle_statuses = self.get_vehicle_statuses_and_lock()

        self.update_user_schedules(user_statuses)
        self.update_vehicle_schedules(vehicle_statuses)
        self.cleanup_status(user_statuses)

        self.update_state_machines(user_statuses, vehicle_statuses)

        self.set_user_statuses_and_unlock(user_statuses)
        self.set_vehicle_statuses_and_unlock(vehicle_statuses)

    def update_state_machines(self, user_statuses, vehicle_statuses):
        current_time = time()

        remove_user_ids = []
        for user_id in user_statuses:
            if user_id not in self.state_machines:
                self.state_machines[user_id] = self.get_state_machine(SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_LOG_IN)

            user_status = user_statuses[user_id]
            target_vehicles = self.relation.get_related(Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME))
            state = self.state_machines[user_id].state

            if len(target_vehicles) == 0:

                if state == SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_LOG_IN:
                    self.state_machines[user_id].wait_user_request(
                        user_id, user_status, USER.STATE.LOG_IN, current_time)
                elif state == SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_REQUEST:
                    self.state_machines[user_id].dispatch(
                        user_id, user_status, SIM_TAXI_USER.STATE.CALLING, current_time)

            elif len(target_vehicles) == 1:
                vehicle_id = target_vehicles[0].id

                if user_id in map(lambda x: x.id, self.vehicle_schedules[vehicle_id][0].targets):
                    vehicle_status = vehicle_statuses[vehicle_id]

                    if state == SIM_TAXI_FLEET.STATE.WAITING_FOR_TAXI_ARRIVE_AT_USER_LOCATION:
                        self.state_machines[user_id].notice(
                            user_id, user_status, vehicle_status,
                            [SIM_TAXI_USER.STATE.WAITING, SIM_TAXI.STATE.STOP_FOR_PICKING_UP], current_time)
                    elif state == SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_GETTING_ON:
                        self.state_machines[user_id].wait_taxi_arrival(
                            user_id, user_status, SIM_TAXI_USER.STATE.GOT_ON, vehicle_id, current_time)
                    elif state == SIM_TAXI_FLEET.STATE.WAITING_FOR_TAXI_ARRIVE_AT_USER_DESTINATION:
                        self.state_machines[user_id].notice(
                            user_id, user_status, vehicle_status,
                            [SIM_TAXI_USER.STATE.MOVING, SIM_TAXI.STATE.STOP_FOR_DISCHARGING], current_time)
                    else:
                        pass
            else:
                logger.error(pformat({"target_vehicles length": target_vehicles}))

            if state == SIM_TAXI_FLEET.STATE.WAITING_FOR_USER_GETTING_OUT:
                if user_status.state in [SIM_TAXI_USER.STATE.GOT_OUT, USER.STATE.LOG_OUT]:
                    remove_user_ids.append(user_id)

        for user_id in remove_user_ids:
            self.relation.remove_relations_of(Target.new_target(user_id, SIM_TAXI_USER.NODE_NAME))
            self.state_machines.pop(user_id)
            user_statuses.pop(user_id)
            self.user_schedules.pop(user_id)

        logger.info(pformat({
            "fleet": dict(map(lambda x: (x[0], x[1].state), self.state_machines.items())),
            "user": dict(map(lambda x: (x[0], x[1].state), user_statuses.items())),
            "vehicle": dict(map(lambda x: (x[0], x[1].state), vehicle_statuses.items())),
        }))
