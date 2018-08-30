#!/usr/bin/env python
# coding: utf-8

import random
from time import time

from ams.helpers import Schedule
from ams.nodes.dispatcher import Helper as DispatcherHelper
from ams.nodes.sim_car import CONST as SIM_CAR


class Helper(DispatcherHelper):

    VEHICLE = SIM_CAR

    @classmethod
    def get_random_location(cls, maps_client, stop_waypoint_ids):
        return random.choice(maps_client.arrow.get_locations_by_waypoint_id(random.choice(stop_waypoint_ids)))

    @classmethod
    def get_random_move_vehicle_schedules(
            cls, start_location, goal_location, stop_waypoint_ids, targets, start_time, maps_client):

        locations = [start_location]
        if 0 < len(stop_waypoint_ids):
            while len(locations) < 3:
                location = cls.get_random_location(maps_client, stop_waypoint_ids)
                if locations[-1] != location:
                    locations.append(location)
        locations.append(goal_location)

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

            shortest_routes = maps_client.route.get_shortest_routes(start_point, goal_points, reverse=False)
            shortest_route = shortest_routes[goal_id]
            shortest_route.pop("cost")
            shortest_route.pop("goal_id")

            schedules.append(Schedule.new_schedule(
                targets,
                cls.VEHICLE.EVENT.MOVE,
                start_time=current_time,
                end_time=current_time + 100,
                route=shortest_route
            ))

            current_time += 100

        schedules.extend([
            Schedule.new_schedule(
                targets=targets,
                event=cls.VEHICLE.EVENT.STOP,
                start_time=current_time,
                end_time=current_time+5,
                route=None
            ),
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
            cls, kvs_client, target_dispatcher, target_vehicle, transportation_status, vehicle_status, vehicle_config,
            dispatcher_config, maps_client):
        print("sim_car_dispatcher.helper.get_vehicle_schedules")
        current_time = time()
        activation_vehicle_schedules = cls.get_activation_vehicle_schedules(transportation_status.targets, current_time)
        transportation_vehicle_schedules = cls.get_random_move_vehicle_schedules(
            vehicle_status.location, vehicle_status.location, dispatcher_config.stop_waypoint_ids,
            transportation_status.targets, activation_vehicle_schedules[-1].period.end, maps_client)
        deactivation_vehicle_schedules = cls.get_deactivation_vehicle_schedules(
            transportation_status.targets, transportation_vehicle_schedules[-1].period.end)
        return activation_vehicle_schedules + transportation_vehicle_schedules + deactivation_vehicle_schedules