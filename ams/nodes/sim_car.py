#!/usr/bin/env python
# coding: utf-8

import json
from sys import float_info
from time import time

from ams import Topic
from ams.nodes import Vehicle, TrafficSignal
from ams.messages import traffic_signal_message

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class SimCar(Vehicle):
    LOWER_INTER_VEHICLE_DISTANCE = 10.0
    LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE = 2.0
    FLOAT_MAX = float_info.max

    def __init__(self, name, waypoint, arrow, route, intersection, waypoint_id, velocity, schedules=None, dt=1.0):
        super().__init__(name, waypoint, arrow, route, waypoint_id, velocity, schedules, dt)

        self.topicTrafficSignalPublish = Topic()
        self.topicTrafficSignalPublish.set_root(TrafficSignal.TOPIC.PUBLISH)
        self.topicTrafficSignalPublish.set_message(traffic_signal_message)

        self.__prevWaypointID = waypoint_id

        self.trafficSignals = {}
        self.otherVehicles = {}

        self.intersection = intersection

        self.add_on_message_function(self.set_other_vehicle_poses)
        self.add_on_message_function(self.set_traffic_signals)

        self.set_subscriber(self.topicVehiclePublish.all)
        self.set_subscriber(self.topicTrafficSignalPublish.all)

    def set_traffic_signals(self, _client, _user_data, topic, message_string):
        root = self.topicVehiclePublish.get_root(topic)
        if root == "traffic_signal":
            message = self.topicVehiclePublish.unserialize(message_string)
            traffic_signal_status = json.loads(message)
            pp(["set_traffic_signals", traffic_signal_status])
            # todo: localize
            entry_exit_route_id = traffic_signal_status["entry_exit_route_id"]
            self.trafficSignals[entry_exit_route_id] = traffic_signal_status

    def set_other_vehicle_poses(self, _client, _user_data, topic, payload):
        if self.topicVehiclePublish.private not in topic and \
                self.topicVehiclePublish.root in topic:
            vehicle_id = self.topicVehiclePublish.getID(topic)
            # print("set_other_vehicle_poses", topic)
            message = self.topicVehiclePublish.unserialize(payload)

            # todo: localize
            self.otherVehicles[vehicle_id] = message

    def get_monitored_route(self, distance=100.0):
        if distance <= 0:
            return None
        arrow_ids = self.schedules[0]["route"]["arrow_ids"]
        arrow_ids = arrow_ids[arrow_ids.index(self.arrow_id):]
        route = {
            "start_waypoint_id": self.waypoint_id,
            "goal_waypoint_id": self.arrow.get_waypoint_ids(self.schedules[0]["route"]["arrow_ids"][-1])[-1],
            "arrow_ids": arrow_ids
        }
        return self.route.get_sliced_route(route, distance)

    def __get_inter_vehicle_distance(self, monitored_route):
        monitored_waypoint_ids = self.route.get_route_waypoint_ids(monitored_route)
        inter_vehicle_distance = SimCar.FLOAT_MAX
        if self.arrow_id is not None and 0 < len(self.otherVehicles):
            other_vehicles_waypoint_ids = list(map(
                lambda x: x["pose"]["position"]["waypoint_id"], self.otherVehicles.values()))
            # print("other_vehicles_waypoint_ids", other_vehicles_waypoint_ids)
            for i, monitored_waypoint_id in enumerate(monitored_waypoint_ids):
                if monitored_waypoint_id in other_vehicles_waypoint_ids:
                    inter_vehicle_distance = self.route.get_distance_of_waypoints(monitored_waypoint_ids[0:i + 1])
                    break
        # print("inter_vehicle_distance {}[m]".format(inter_vehicle_distance))
        return inter_vehicle_distance

    def __get_inter_traffic_signal_distance(self, monitored_route):
        route_arrow_ids = monitored_route["arrow_ids"]
        inter_traffic_signal_distance = SimCar.FLOAT_MAX

        intersection_ids = list(set(map(lambda x: x["intersectionID"], self.trafficSignals.values())))
        entry_exit_route_ids = []
        for intersectionID in intersection_ids:
            entry_exit_route_ids.extend(
                self.intersection.getEntryExitRouteIDsInArrowIDs(intersectionID, route_arrow_ids))

        min_arrow_id_index = len(route_arrow_ids)
        min_goal_waypoint_id = None
        for entry_exit_route_id in entry_exit_route_ids:
            if self.trafficSignals[entry_exit_route_id]["state"] in [
                    TrafficSignal.STATE.YELLOW, TrafficSignal.STATE.RED]:
                entry_exit_route_arrow_ids = self.intersection.get_arrow_ids_of_entry_exit_route(
                    intersectionID, entry_exit_route_id)
                entry_arrow_id = entry_exit_route_arrow_ids[0]
                # print(self.trafficSignals[entry_exit_route_id]["state"], entry_arrow_id)

                entry_arrow_id_index = route_arrow_ids.index(entry_arrow_id)
                if entry_arrow_id_index < min_arrow_id_index:
                    min_arrow_id_index = entry_arrow_id_index
                    border_point = self.intersection.getBorderPoint(intersectionID, entry_arrow_id)
                    min_goal_waypoint_id = border_point["prev_waypoint_id"]

        if min_goal_waypoint_id is not None:
            min_route = {
                "start_waypoint_id": monitored_route["start_waypoint_id"],
                "goal_waypoint_id": min_goal_waypoint_id,
                "arrow_ids": route_arrow_ids[:min_arrow_id_index+1],
            }
            inter_traffic_signal_distance = self.route.get_route_length(min_route)

        # print("inter_traffic_signal_distance {}[m]".format(inter_traffic_signal_distance))
        return inter_traffic_signal_distance

    def __get_movable_distance(self):
        movable_distance = SimCar.FLOAT_MAX
        if 0 < len(self.schedules):
            if self.schedules[0]["action"] == Vehicle.ACTION.MOVE:
                # check inter-vehicle distance
                monitored_route = self.get_monitored_route()
                if monitored_route is None:
                    return 0.0
                inter_vehicle_distance = self.__get_inter_vehicle_distance(monitored_route)
                movable_distance = inter_vehicle_distance - SimCar.LOWER_INTER_VEHICLE_DISTANCE

                # check inter-trafficSignal distance
                monitored_route = self.get_monitored_route(movable_distance)
                if monitored_route is None:
                    return 0.0
                inter_traffic_signal_distance = self.__get_inter_traffic_signal_distance(monitored_route)
                movable_distance = min(
                    movable_distance, inter_traffic_signal_distance - SimCar.LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE)

        return movable_distance

    def update_pose(self):
        movable_distance = self.__get_movable_distance()
        # print("movable_distance", movable_distance)
        delta_distance = min(self.velocity * self.dt, movable_distance)
        if 0.0 < delta_distance:
            self.__prevWaypointID = self.waypoint_id
            self.lat, self.lng, self.yaw, self.arrow_id, self.waypoint_id = self.get_next_pose(delta_distance)

    def is_achieved(self):
        goal_waypoint_id = self.schedules[0]["route"]["goal"]["waypoint_id"]
        # get pass waypoint_ids
        waypoint_ids = []
        for arrow_id in self.schedules[0]["route"]["arrow_ids"][0:2]:
            waypoint_ids.extend(self.arrow.get_waypoint_ids(arrow_id))
        pass_waypoint_ids = \
            waypoint_ids[waypoint_ids.index(self.__prevWaypointID):waypoint_ids.index(self.waypoint_id) + 1]

        return goal_waypoint_id in pass_waypoint_ids

    def get_next_pose(self, delta_distance):
        arrows, to_arrows, _ = self.arrow.get_arrow_ids_to_arrows(
            self.schedules[0]["route"]["arrow_ids"][0:2])
        lat, lng, arrow_id = self.arrow.get_advanced_latlng_in_arrows(
            self.lat, self.lng, delta_distance, arrows=arrows, next_arrows=to_arrows)
        waypoint_id, lat, lng, _ = self.arrow.get_point_to_arrow(lat, lng, arrow_id)
        heading = self.arrow.get_heading(arrow_id, waypoint_id)
        return lat, lng, heading, arrow_id, waypoint_id

    def update_status(self):
        current_time = time()
        if self.state == Vehicle.STATE.STOP:
            print("state:stop action:" + str(self.schedules[0]["action"]))
            if self.schedules[0]["action"] == Vehicle.ACTION.MOVE:
                self.state = Vehicle.STATE.MOVE
            else:
                if self.schedules[0]["start_time"]+self.schedules[0]["duration"] <= current_time:
                    if 1 < len(self.schedules):
                        self.schedules.pop(0)

                        # update next schedule
                        dif_time = current_time - self.schedules[0]["start_time"]
                        self.schedules[0]["start_time"] += dif_time
                        self.schedules[0]["duration_time"] = dif_time

                        print(self.schedules[0])
                        self.state = Vehicle.STATE.MOVE

        elif self.state == Vehicle.STATE.MOVE:
            self.update_pose()
            if self.is_achieved():
                print("*** arrival ***")
                self.waypoint_id = self.schedules[0]["route"]["goal"]["waypoint_id"]
                self.lat, self.lng = self.waypoint.get_latlng(self.waypoint_id)
                self.yaw = self.arrow.get_heading(self.arrow_id, self.waypoint_id)
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0]["start_time"]
                self.schedules[0]["start_time"] += dif_time
                self.schedules[0]["duration_time"] = dif_time

                self.state = Vehicle.STATE.STOP
            else:
                arrow_ids = self.schedules[0]["route"]["arrow_ids"]
                self.schedules[0]["route"]["arrow_ids"] = arrow_ids[arrow_ids.index(self.arrow_id):]
