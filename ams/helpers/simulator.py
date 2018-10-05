#!/usr/bin/env python
# coding: utf-8

from sys import float_info
import math

from ams import logger
from ams.structures import Pose, Autoware
from ams.helpers import Position, Vector, Route


class Simulator(object):

    @staticmethod
    def search_closest_waypoint_from_lane_array(pose_stamped, lane_array, limit_distance=3.0):
        distance_min = float_info.max
        i = 0
        closest_waypoint = Autoware.ROSMessage.ClosestWaypoint.new_data(**{
            "lane_array_id": lane_array.id, "index": -1})
        for lane in lane_array.lanes:
            for waypoint in lane.waypoints:
                distance = math.sqrt(sum(map(
                    lambda x: math.pow(pose_stamped.pose.position[x] - waypoint.pose.pose.position[x], 2.0),
                    ["x", "y", "z"]
                )))
                if distance < distance_min:
                    distance_min = distance
                    if distance_min <= limit_distance:
                        closest_waypoint.index = i
                i += 1
        return closest_waypoint

    @staticmethod
    def update_pose(
            maps_client, vehicle_status, route, traffic_signals, other_vehicle_locations, dt=1.0,
            lower_inter_vehicle_distance=10.0, lower_inter_traffic_signal_distance=2.0
    ):
        locations = maps_client.route.get_locations(route)
        if vehicle_status.location not in locations:
            if vehicle_status.location.waypoint_id == locations[0].waypoint_id:
                forward_locations = locations[:]
            else:
                raise ValueError(vehicle_status, locations)
        else:
            forward_locations = locations[locations.index(vehicle_status.location):]

        movable_distance = Simulator.get_movable_distance(
            maps_client, forward_locations, traffic_signals, other_vehicle_locations,
            lower_inter_vehicle_distance, lower_inter_traffic_signal_distance)

        delta_distance = min(vehicle_status.velocity * dt, movable_distance)

        if 0.0 < delta_distance:
            vehicle_status.pose, vehicle_status.location = \
                Simulator.get_next_pose_and_location(
                    maps_client, vehicle_status.pose.position, delta_distance, forward_locations)

    @staticmethod
    def get_movable_distance(
            maps_client, locations, traffic_signals, other_vehicle_locations,
            lower_inter_vehicle_distance=10.0, lower_inter_traffic_signal_distance=2.0):

        monitored_locations = Simulator.get_monitored_locations(maps_client, locations)

        distance_from_preceding_vehicle = Simulator.get_distance_from_preceding_vehicle(
            maps_client, other_vehicle_locations, monitored_locations)
        movable_distance = distance_from_preceding_vehicle - lower_inter_vehicle_distance

        monitored_locations = Simulator.get_monitored_locations(maps_client, locations, movable_distance)

        distance_from_stopline = \
            Simulator.get_distance_from_stopline(maps_client, traffic_signals, monitored_locations)

        movable_distance = min(movable_distance, distance_from_stopline - lower_inter_traffic_signal_distance)

        return movable_distance

    @staticmethod
    def get_monitored_locations(maps_client, locations, upper_distance=100.0):
        position_vectors = \
            list(map(lambda x: Position.get_vector(maps_client.waypoint.get_position(x.waypoint_id)), locations))

        monitored_locations = [locations[0]]
        distance_sum = 0.0
        for i in range(1, len(position_vectors)):
            distance_sum += Vector.get_norm(Vector.get_sub_vector(position_vectors[i-1], position_vectors[i]))
            if upper_distance < distance_sum:
                break
            monitored_locations.append(locations[i])

        return monitored_locations

    @staticmethod
    def get_distance_from_preceding_vehicle(maps_client, other_vehicle_locations, monitored_locations):
        distance_from_preceding_vehicle = float_info.max
        if 0 < len(other_vehicle_locations):
            other_vehicles_waypoint_ids = list(map(
                lambda x: x.waypoint_id, other_vehicle_locations.values()))
            for i, monitored_location in enumerate(monitored_locations):
                if monitored_location.waypoint_id in other_vehicles_waypoint_ids:
                    distance_from_preceding_vehicle = \
                        maps_client.route.get_distance_of_waypoints(
                            list(map(lambda x: x.waypoint_id, monitored_locations[:i + 1])))
                    break
        if distance_from_preceding_vehicle < float_info.max:
            logger.info("distance_from_preceding_vehicle {}[m]".format(distance_from_preceding_vehicle))
        return distance_from_preceding_vehicle

    @staticmethod
    def get_distance_from_stopline(maps_client, traffic_signals, monitored_locations):
        distance_from_stopline = float_info.max

        not_green_traffic_signal_locations_set = list(map(
            lambda x: maps_client.route.get_locations(Route.decode(x.route_code)), filter(
                lambda x: x.state in [x.CONST.STATE.YELLOW, x.CONST.STATE.RED],
                traffic_signals.values())))

        for not_green_traffic_signal_locations in not_green_traffic_signal_locations_set:
            if not_green_traffic_signal_locations[0] in monitored_locations:
                index_min = min(
                    len(not_green_traffic_signal_locations),
                    len(monitored_locations[monitored_locations.index(not_green_traffic_signal_locations[0]):])
                )
                if not_green_traffic_signal_locations[:index_min] in monitored_locations:
                    monitored_locations = \
                        monitored_locations[:monitored_locations.index(not_green_traffic_signal_locations[0])]
                    distance_from_stopline = \
                        maps_client.route.get_distance_of_waypoints(
                            list(map(lambda x: x.waypoint_id, monitored_locations)))

        if distance_from_stopline < float_info.max:
            logger.info("distance_from_stopline {}[m]".format(distance_from_stopline))
        return distance_from_stopline

    @staticmethod
    def get_next_pose_and_location(maps_client, position, distance, locations):
        next_position = position
        next_location = locations[0]

        position_vectors = \
            list(map(lambda x: Position.get_vector(maps_client.waypoint.get_position(x.waypoint_id)), locations))

        distance_sum = 0.0
        for i in range(1, len(position_vectors)):
            if i == 1:
                d = Vector.get_norm(Vector.get_sub_vector(
                    Position.get_vector(maps_client.waypoint.get_position(locations[1].waypoint_id)),
                    Position.get_vector(position)
                ))
            else:
                d = Vector.get_norm(Vector.get_sub_vector(position_vectors[i-1], position_vectors[i]))

            if distance < distance_sum + d:
                v12 = Vector.get_sub_vector(position_vectors[i], position_vectors[i-1])
                next_position = Position.new_position(*Vector.get_add_vector(
                    position_vectors[i-1],
                    Vector.get_div_vector(Vector.get_mul_vector([d]*len(v12), v12), [Vector.get_norm(v12)]*len(v12))
                ))
                next_location = locations[i-1]
                break
            if i == len(position_vectors)-1:
                next_position = Position.new_position(*position_vectors[-1])
                next_location = locations[-1]
                break
            distance_sum += d

        next_pose = Pose.new_data(
            position=next_position,
            orientation=maps_client.arrow.get_orientation(next_location.arrow_code, next_location.waypoint_id)
        )
        next_location.geohash = maps_client.waypoint.get_geohash(next_location.waypoint_id)

        return next_pose, next_location

    @staticmethod
    def update_velocity(maps_client, vehicle_status, dt=1.0, acceleration_max=0.3):
        speed_limit = maps_client.waypoint.get_speed_limit(vehicle_status.location.waypoint_id)
        if vehicle_status.velocity < speed_limit:
            vehicle_status.velocity += min(acceleration_max * dt, speed_limit - vehicle_status.velocity)
        elif speed_limit < vehicle_status.velocity:
            vehicle_status.velocity = speed_limit
        return
