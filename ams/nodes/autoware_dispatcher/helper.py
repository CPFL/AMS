#!/usr/bin/env python
# coding: utf-8

import random

from ams.helpers import Schedule
from ams.nodes.dispatcher import Helper as DispatcherHelper
from ams.nodes.autoware import CONST as AUTOWARE
from ams.nodes.autoware import Structure as AutowareStructure
from ams.nodes.autoware_dispatcher import Structure


class Helper(DispatcherHelper):

    VEHICLE = AUTOWARE
    VehicleStructure = AutowareStructure
    Structure = Structure

    @classmethod
    def get_random_location(cls, clients, stop_waypoint_ids):
        return random.choice(clients["maps"].arrow.get_locations_by_waypoint_id(random.choice(stop_waypoint_ids)))

    @classmethod
    def get_transportation_vehicle_schedules(
            cls, clients, targets, start_time=None, start_location=None, goal_location=None, stop_waypoint_ids=None):

        locations = [start_location]
        if 0 < len(stop_waypoint_ids):
            # while len(locations) < 3:
            while len(locations) < 2:
                location = cls.get_random_location(clients, stop_waypoint_ids)
                if locations[-1] != location:
                    locations.append(location)


        current_time = start_time
        schedules = [
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.START_MISSION,
                start_time=current_time,
                end_time=current_time + 10,
                route=None
            )
        ]
        current_time += 10
        for i in range(1, len(locations)):
            start_point = {
                "waypoint_id": locations[i - 1].waypoint_id,
                "arrow_code": locations[i - 1].arrow_code
            }
            goal_id = "route" + locations[i].waypoint_id
            goal_points = [{
                "goal_id": goal_id,
                "waypoint_id": locations[i].waypoint_id,
                "arrow_code": locations[i].arrow_code
            }]

            shortest_routes = clients["maps"].route.get_shortest_routes(start_point, goal_points, reverse=False)
            shortest_route = shortest_routes[goal_id]
            shortest_route.pop("cost")
            shortest_route.pop("goal_id")

            schedules.extend([
                Schedule.new_schedule(
                    targets,
                    cls.VEHICLE.EVENT.SEND_LANE_WAYPOINT_ARRAY,
                    start_time=current_time,
                    end_time=current_time + 100,
                    route=shortest_route
                ),
                Schedule.new_schedule(
                    targets,
                    cls.VEHICLE.EVENT.SEND_ENGAGE,
                    start_time=current_time,
                    end_time=current_time + 100,
                    route=None
                )
            ])

            current_time += 100

        schedules.extend([
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.END_MISSION,
                start_time=current_time+5,
                end_time=current_time+5,
                route=None
            )
        ])

        return schedules

    @classmethod
    def get_vehicle_schedules(
            cls, clients, target_roles, transportation_status, vehicle_status, vehicle_config, dispatcher_config):

        activation_vehicle_schedules = cls.get_activation_vehicle_schedules(
            clients, transportation_status.targets, Schedule.get_time())

        transportation_vehicle_schedules = cls.get_transportation_vehicle_schedules(
            clients, transportation_status.targets, activation_vehicle_schedules[-1].period.end,
            vehicle_status.location, vehicle_status.location, dispatcher_config.stop_waypoint_ids)

        deactivation_vehicle_schedules = cls.get_deactivation_vehicle_schedules(
            clients, transportation_status.targets, transportation_vehicle_schedules[-1].period.end)

        return activation_vehicle_schedules + transportation_vehicle_schedules + deactivation_vehicle_schedules
