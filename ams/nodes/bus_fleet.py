#!/usr/bin/env python
# coding: utf-8

from time import time

from ams import Topic, Route, Schedule, Target, Relation, SelectiveRoute
from ams.nodes import FleetManager, User, TaxiUser, Vehicle, SimBus
from ams.messages import VehicleStatus
from ams.structures import Schedules
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class BusFleet(FleetManager):

    DISPATCHABLE_GEOHASH_DIGIT = 6
    TIMEOUT = 30.0
    UNASSIGNED_ROUTE = {"unassigned": "bus_route"}

    class TOPIC(object):
        SUBSCRIBE = "sub_bus_fleet"

    def __init__(self, name, waypoint, arrow, route, spot):
        super().__init__(name, waypoint, arrow, route)

        self.topicVehicleStatus = Topic()
        self.topicVehicleStatus.set_root(Vehicle.TOPIC.PUBLISH)

        self.topicVehicleSchedules = Topic()
        self.topicVehicleSchedules.set_root(Vehicle.TOPIC.SUBSCRIBE)

        self.topicBusSchedules = Topic()
        self.topicBusSchedules.set_root(BusFleet.TOPIC.SUBSCRIBE)

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.spot = spot
        self.relation = Relation()
        self.__bus_routes = {}

        self.bus_parkable_spots = self.spot.get_spots_of_target_group(Target.new_node_target(SimBus))
        # self.bus_parkable_spots.update(self.spot.get_spots_of_target_node(AutowareBus))

        self.vehicle_statuses = {}
        self.vehicle_schedules = {}
        self.bus_schedules = {}
        self.bus_stop_spots = {}

        self.add_on_message_function(self.update_vehicle_status)
        # self.add_on_message_function(self.update_bus_schedules)

        self.set_subscriber(self.topicVehicleStatus.all)
        # self.set_subscriber(self.topicBusSchedules.all)

    def set_bus_routes(self, bus_routes):
        self.__bus_routes = bus_routes
        for bus_route_id in self.__bus_routes:
            self.relation.add_relation(
                Target.new_target(bus_route_id, "bus_route"),
                BusFleet.UNASSIGNED_ROUTE
            )

    # def update_bus_schedules(self, _client, _userdata, topic, payload):
    #     if self.topicBusSchedules.root+"/schedules" in topic:
    #         vehicle_id = self.topicVehicleStatus.get_id(topic)
    #         bus_schedule = Schedules.new_data(self.topicBusSchedules.unserialize(payload))
    #         self.bus_schedules[vehicle_id] = [bus_schedule]
    #         self.update_bus_stop_spots(vehicle_id)
    #
    # def update_bus_stop_spots(self, vehicle_id):
    #     arrow_waypoint_array = list(set(map(lambda x: self.route.get_arrow_waypoint_array(x.route), self.vehicle_statuses[vehicle_id])))
    #     self.bus_stop_spots[vehicle_id] = dict(filter(
    #         lambda x: (x[1].contact.waypoint_id, x[1].contact.arrow_code) in arrow_waypoint_array,
    #         self.bus_parkable_spots.items()
    #     ))

    def update_vehicle_status(self, _client, _userdata, topic, payload):
        if self.topicVehicleStatus.root in topic:
            vehicle_id = self.topicVehicleStatus.get_id(topic)
            vehicle_status = VehicleStatus.new_data(**self.topicVehicleStatus.unserialize(payload))
            if vehicle_id not in self.vehicle_schedules:
                self.vehicle_schedules[vehicle_id] = [vehicle_status.schedule]
            else:
                if self.vehicle_statuses[vehicle_id].state != vehicle_status.state:
                    self.update_vehicle_schedules(vehicle_id, vehicle_status)
            self.vehicle_statuses[vehicle_id] = vehicle_status

    def update_vehicle_schedules(self, vehicle_id, vehicle_status):
        self.vehicle_schedules[vehicle_id].pop(0)
        self.vehicle_schedules[vehicle_id][0] = vehicle_status.schedule

    def get_unassined_bus_route_id(self):
        target_bus_routes = self.relation.get_related(BusFleet.UNASSIGNED_ROUTE)
        if len(target_bus_routes) == 0:
            return None
        return target_bus_routes[0]["id"]

    def get_to_circular_route(self, start_waypoint_id, start_arrow_code, bus_route_id):
        start_point = {
            "arrow_code": start_arrow_code,
            "waypoint_id": start_waypoint_id
        }
        goal_points = []
        for selective_route in self.__bus_routes[bus_route_id]["selective_routes"]:
            for main_route_node_arrow_waypoint in \
                    self.route.get_route_node_arrow_waypoint_array(selective_route.main_route):
                goal_point = {
                    "goal_id": main_route_node_arrow_waypoint["waypoint_id"],
                    "waypoint_id": main_route_node_arrow_waypoint["waypoint_id"],
                    "arrow_code": main_route_node_arrow_waypoint["arrow_code"],
                }
                if goal_point not in goal_points:
                    goal_points.append(goal_point)
        shortest_routes = self.route.get_shortest_routes(start_point, goal_points, reverse=False)
        to_circular_route = min(shortest_routes.items(), key=lambda x: x[1]["cost"])[1]
        to_circular_route.pop("cost")
        to_circular_route.pop("goal_id")
        return to_circular_route

    def get_to_bus_stop_route(self, start_waypoint_id, start_arrow_code, bus_route_id):
        for selective_route in self.__bus_routes[bus_route_id]["selective_routes"]:
            if start_arrow_code in SelectiveRoute.get_arrow_codes(selective_route):
                arrow_codes = selective_route.sub_routes[0].arrow_codes
                return Route.new_route(
                    start_waypoint_id,
                    selective_route.sub_routes[0].goal_waypoint_id,
                    arrow_codes[arrow_codes.index(start_arrow_code):]
                )

    # def get_to_parking_route(self):

    def get_vehicle_schedules(self, vehicle_id, schedule_type, start_time):
        vehicle_status = self.vehicle_statuses[vehicle_id]
        if schedule_type == SimBus.STATE.MOVE_TO_CIRCULAR_ROUTE:
            bus_route_id = self.get_unassined_bus_route_id()
            if bus_route_id is None:
                return None

            to_circular_route = self.get_to_circular_route(
                vehicle_status.location.waypoint_id,  vehicle_status.location.arrow_code, bus_route_id)

            pp(("to_circular_route", to_circular_route))

            target_vehicle = Target.new_target(vehicle_id, "SimBus")
            vehicle_schedules = Schedule.get_merged_schedules([], [Schedule.new_schedule(
                [target_vehicle], Vehicle.ACTION.MOVE, start_time, start_time+1000, to_circular_route
            )])

            to_bus_stop_route = self.get_to_bus_stop_route(
                to_circular_route.start_waypoint_id, to_circular_route.arrow_codes[0], bus_route_id)

            pp(("to_bus_stop_route", to_bus_stop_route, to_circular_route.start_waypoint_id, to_circular_route.arrow_codes[0], bus_route_id))

            vehicle_schedules = Schedule.get_merged_schedules(vehicle_schedules, [Schedule.new_schedule(
                [target_vehicle], Vehicle.ACTION.MOVE, start_time, start_time+1000, to_bus_stop_route
            )])

            vehicle_schedules = Schedule.get_merged_schedules(vehicle_schedules, [Schedule.new_schedule(
                [target_vehicle], Vehicle.ACTION.STOP, start_time, start_time+86400,
                Route.new_point_route(to_bus_stop_route.goal_waypoint_id, to_bus_stop_route.arrow_codes[-1]))])


        return vehicle_schedules

    def update_status(self):
        for vehicle_id, vehicle_status in self.vehicle_statuses.items():
            if vehicle_status.state == SimBus.STATE.STANDBY:
                if self.vehicle_schedules[vehicle_id][0].event != SimBus.STATE.MOVE_TO_CIRCULAR_ROUTE:
                    vehicle_schedules = self.get_vehicle_schedules(
                        vehicle_id, SimBus.STATE.MOVE_TO_CIRCULAR_ROUTE, time())
                    if vehicle_schedules is not None:
                        vehicle_schedules[0].event = SimBus.STATE.MOVE_TO_CIRCULAR_ROUTE
                        self.vehicle_schedules[vehicle_id] = vehicle_schedules
                        payload = self.topicVehicleSchedules.serialize(self.vehicle_schedules[vehicle_id])
                        self.publish(self.topicVehicleSchedules.root + "/" + vehicle_id + "/schedules", payload)

            elif vehicle_status.state == SimBus.STATE.MOVE_TO_CIRCULAR_ROUTE:
                pass
            elif vehicle_status.state == SimBus.STATE.MOVE_TO_BUS_STOP:
                pass
            elif vehicle_status.state == SimBus.STATE.STOP_TO_DISCHARGE:
                pass
            elif vehicle_status.state == SimBus.STATE.STOP_TO_TAKE_UP:
                pass
            elif vehicle_status.state == SimBus.STATE.STOP_TO_DISCHARGING_AND_TAKE_UP:
                pass
            elif vehicle_status.state == SimBus.STATE.MOVE_TO_PARKING:
                pass
            elif vehicle_status.state == SimBus.STATE.STOP_TO_PARK:
                pass
            else:
                print("unknown sim bus state", vehicle_status.state)

        return
