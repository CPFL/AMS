#!/usr/bin/env python
# coding: utf-8

from time import time
from copy import deepcopy

from transitions import Machine

from ams import logger, Topic, Route, Target
from ams.nodes import Vehicle
from ams.messages import TrafficSignalStatus
from ams.structures import SIM_CAR, TRAFFIC_SIGNAL, Location


class SimCar(Vehicle):

    CONST = SIM_CAR

    def __init__(self, _id, name, waypoint, arrow, route, intersection, dt=1.0):
        super().__init__(_id, name, waypoint, arrow, route, dt=dt)

        self.state_machine = self.get_state_machine(SIM_CAR.STATE.STOP)
        self.velocity = None

        self.traffic_signals = self.manager.dict()
        self.traffic_signals_lock = self.manager.Lock()
        self.other_vehicle_locations = self.manager.dict()
        self.other_vehicle_locations_lock = self.manager.Lock()
        self.intersection = intersection

        self.__topicPubLocation = Topic()
        self.__topicPubLocation.set_targets(Target.new_target(self.target.id, SIM_CAR.NODE_NAME))
        self.__topicPubLocation.set_categories(SIM_CAR.TOPIC.CATEGORIES.LOCATION)

        self.__topicSubStatus = Topic()
        self.__topicSubStatus.set_targets(Target.new_target(None, SIM_CAR.NODE_NAME), None)
        self.__topicSubStatus.set_categories(SIM_CAR.TOPIC.CATEGORIES.LOCATION)
        self.__topicSubStatus.set_message(Location)
        self.set_subscriber(self.__topicSubStatus, self.update_other_vehicle_locations)

        self.__topicSubTrafficSignalStatus = Topic()
        self.__topicSubTrafficSignalStatus.set_targets(Target.new_target(None, TRAFFIC_SIGNAL.NODE_NAME), None)
        self.__topicSubTrafficSignalStatus.set_categories(TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS)
        self.__topicSubTrafficSignalStatus.set_message(TrafficSignalStatus)
        self.set_subscriber(self.__topicSubTrafficSignalStatus, self.update_traffic_signals)

    def set_velocity(self, velocity):
        self.velocity = velocity

    def publish_location(self):
        self.status.location.geohash = self.waypoint.get_geohash(self.status.location.waypoint_id)
        payload = self.__topicPubLocation.serialize(self.status.location)
        self.publish(self.__topicPubLocation, payload)

    def update_traffic_signals(self, _client, _user_data, _topic, payload):
        # todo: localize
        traffic_signal_status = self.__topicSubTrafficSignalStatus.unserialize(payload)

        self.traffic_signals_lock.acquire()
        self.traffic_signals[traffic_signal_status.route_code] = traffic_signal_status
        self.traffic_signals_lock.release()

    def update_other_vehicle_locations(self, _client, _user_data, topic, payload):
        # todo: localize
        from_id = Topic.get_from_id(topic)
        if self.target.id != from_id:
            self.other_vehicle_locations_lock.acquire()
            self.other_vehicle_locations[from_id] = self.__topicSubStatus.unserialize(payload)
            self.other_vehicle_locations_lock.release()

    def get_monitored_route(self, distance=100.0):
        if distance <= 0:
            return None
        arrow_codes = self.status.schedule.route.arrow_codes
        arrow_codes = arrow_codes[arrow_codes.index(self.status.location.arrow_code):]
        route = Route.new_route(
            self.status.location.waypoint_id,
            self.arrow.get_waypoint_ids(self.status.schedule.route.arrow_codes[-1])[-1],
            arrow_codes)
        return self.route.get_sliced_route(route, distance)

    def get_distance_from_preceding_vehicle(self, monitored_route):
        self.other_vehicle_locations_lock.acquire()
        other_vehicle_locations = deepcopy(self.other_vehicle_locations)
        self.other_vehicle_locations_lock.release()

        monitored_waypoint_ids = self.route.get_waypoint_ids(monitored_route)
        distance_from_preceding_vehicle = SIM_CAR.FLOAT_MAX
        if self.status.location.arrow_code is not None and 0 < len(other_vehicle_locations):
            other_vehicles_waypoint_ids = list(map(
                lambda x: x.waypoint_id, other_vehicle_locations.values()))
            for i, monitored_waypoint_id in enumerate(monitored_waypoint_ids):
                if monitored_waypoint_id in other_vehicles_waypoint_ids:
                    distance_from_preceding_vehicle = \
                        self.route.get_distance_of_waypoints(monitored_waypoint_ids[:i+1])
                    break
        if distance_from_preceding_vehicle < SIM_CAR.FLOAT_MAX:
            logger.info("distance_from_preceding_vehicle {}[m]".format(distance_from_preceding_vehicle))
        return distance_from_preceding_vehicle

    def get_distance_from_stopline(self, monitored_route):
        monitored_arrow_codes = monitored_route.arrow_codes
        distance_from_stopline = SIM_CAR.FLOAT_MAX

        self.traffic_signals_lock.acquire()
        traffic_signals = deepcopy(self.traffic_signals)
        self.traffic_signals_lock.release()

        not_green_traffic_signal_route_codes = list(map(
            lambda x: x.route_code, filter(
                lambda x: x.state in [TRAFFIC_SIGNAL.STATE.YELLOW, TRAFFIC_SIGNAL.STATE.RED],
                traffic_signals.values())))

        new_monitored_route = None
        for i, monitored_arrow_code in enumerate(monitored_arrow_codes):
            for not_green_traffic_signal_route_code in not_green_traffic_signal_route_codes:
                if monitored_arrow_code in not_green_traffic_signal_route_code:
                    not_green_traffic_signal_route = Route.decode_route_code(not_green_traffic_signal_route_code)
                    if monitored_arrow_code == not_green_traffic_signal_route.arrow_codes[0]:
                        waypoint_ids = self.arrow.get_waypoint_ids(monitored_arrow_code)
                        if self.status.location.waypoint_id not in waypoint_ids or \
                                waypoint_ids.index(self.status.location.waypoint_id) <= waypoint_ids.index(
                                    not_green_traffic_signal_route.start_waypoint_id):
                            new_monitored_route = Route.new_route(
                                monitored_route.start_waypoint_id,
                                not_green_traffic_signal_route.start_waypoint_id,
                                monitored_arrow_codes[:i+1])
                            break
            if new_monitored_route is not None:
                break

        if new_monitored_route is not None:
            distance_from_stopline = self.route.get_route_length(new_monitored_route)

        if distance_from_stopline < SIM_CAR.FLOAT_MAX:
            logger.info("distance_from_stopline {}[m]".format(distance_from_stopline))
        return distance_from_stopline

    def __get_movable_distance(self):
        monitored_route = self.get_monitored_route()
        if monitored_route is None:
            return 0.0
        distance_from_preceding_vehicle = self.get_distance_from_preceding_vehicle(monitored_route)
        movable_distance = distance_from_preceding_vehicle - SIM_CAR.LOWER_INTER_VEHICLE_DISTANCE

        monitored_route = self.get_monitored_route(movable_distance)
        if monitored_route is None:
            return 0.0
        distance_from_stopline = self.get_distance_from_stopline(monitored_route)
        movable_distance = min(
            movable_distance, distance_from_stopline - SIM_CAR.LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE)

        return movable_distance

    def update_pose(self):
        movable_distance = self.__get_movable_distance()
        delta_distance = min(self.velocity * self.dt, movable_distance)
        if 0.0 < delta_distance:
            self.np_position, self.status.pose.orientation.rpy.yaw,\
                self.status.location.arrow_code, self.status.location.waypoint_id = \
                self.get_next_pose(delta_distance, self.status.schedule.route)
        self.publish_location()

    def update_route(self):
        index = self.status.schedule.route.arrow_codes.index(self.status.location.arrow_code)
        self.status.schedule.route.arrow_codes[:] = self.status.schedule.route.arrow_codes[index:]
        self.status.schedule.route.start_waypoint_id = self.status.location.waypoint_id

    def update_velocity(self):
        speed_limit = self.waypoint.get_speed_limit(self.status.location.waypoint_id)
        if self.velocity < speed_limit:
            self.velocity += min(SIM_CAR.ACCELERATION_MAX * self.dt, speed_limit - self.velocity)
        elif speed_limit < self.velocity:
            self.velocity = speed_limit
        return

    def get_next_pose(self, delta_distance, route):
        position, waypoint_id, arrow_code = self.route.get_moved_position(
            self.np_position, delta_distance, route)
        yaw = self.arrow.get_yaw(arrow_code, waypoint_id)
        return position, yaw, arrow_code, waypoint_id

    def update_pose_to_route_start(self):
        self.status.location.waypoint_id = self.status.schedule.route.start_waypoint_id
        self.status.location.arrow_code = self.status.schedule.route.arrow_codes[0]
        self.np_position = self.waypoint.get_np_position(self.status.location.waypoint_id)
        self.status.pose.orientation.rpy.yaw = \
            self.arrow.get_yaw(self.status.location.arrow_code, self.status.location.waypoint_id)
        self.velocity = 0.0

    def get_state_machine(self, initial_state):
        machine = Machine(
            states=list(SIM_CAR.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": SIM_CAR.TRIGGER.MOVE, "source": SIM_CAR.STATE.STOP, "dest": SIM_CAR.STATE.MOVE,
                "conditions": [self.after_state_change_update_schedules]
            },
            {
                "trigger": SIM_CAR.TRIGGER.MOVE, "source": SIM_CAR.STATE.MOVE, "dest": SIM_CAR.STATE.MOVE,
                "conditions": [self.condition_achieved_and_update_schedules]
            },
            {
                "trigger": SIM_CAR.TRIGGER.STOP, "source": SIM_CAR.STATE.MOVE, "dest": SIM_CAR.STATE.STOP,
                "conditions": [self.condition_achieved_and_update_schedules]
            }
        ])
        return machine

    def condition_achieved(self):
        return self.status.location.waypoint_id == self.status.schedule.route.goal_waypoint_id

    def condition_time_limit(self, current_time):
        return self.status.schedule.period.end < current_time

    def after_state_change_update_schedules(self, current_time, schedules):
        schedules[:] = self.get_next_schedules(schedules, current_time)
        self.update_pose_to_route_start()
        return True

    def condition_achieved_and_update_schedules(self, current_time, schedules):
        if self.condition_achieved():
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        return False

    def condition_time_limit_and_update_schedules(self, current_time, schedules):
        if self.condition_time_limit(current_time):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        return False

    def update_status(self):
        schedules = self.get_schedules_and_lock()

        if self.state_machine.state == SIM_CAR.STATE.MOVE:
            self.update_pose()
            self.update_route()
            self.update_velocity()

        if 1 < len(schedules):
            current_time = time()
            next_event = schedules[1].event

            if next_event == SIM_CAR.TRIGGER.MOVE:
                self.state_machine.move(current_time, schedules)
            elif next_event == SIM_CAR.TRIGGER.STOP:
                self.state_machine.stop(current_time, schedules)
            else:
                pass

        self.set_schedules_and_unlock(schedules)
