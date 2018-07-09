#!/usr/bin/env python
# coding: utf-8

from ams.structures import KVS_CLIENT
from ams.helpers import Target, Location
from ams.nodes.vehicle import Helper as VehicleHelper
from ams.nodes.traffic_signal import CONST as TRAFFIC_SIGNAL
from ams.nodes.sim_car import CONST


class Helper(VehicleHelper):

    VEHICLE = CONST

    @classmethod
    def get_traffic_signal_status_key(cls, target_vehicle, target_traffic_signal):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_vehicle), Target.get_code(target_traffic_signal), "status"])

    @classmethod
    def get_vehicle_location_key(cls, target_vehicle):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([Target.get_code(target_vehicle), "location"])

    @classmethod
    def get_all_traffic_signal_status_keys(cls, kvs_client, target_vehicle):
        return kvs_client.keys(KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_vehicle),
            Target.get_code(Target.new_target(group=TRAFFIC_SIGNAL.NODE_NAME)),
            "status"
        ]))

    @classmethod
    def get_other_vehicle_location_keys(cls, kvs_client, target_vehicle):
        all_keys = kvs_client.keys(KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            cls.VEHICLE.NODE_NAME, "+", "location"
        ]))
        return list(filter(lambda x: x.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[1] != target_vehicle.id, all_keys))

    @classmethod
    def get_traffic_signal_status(cls, kvs_client, traffic_signal_status_key):
        return kvs_client.get(traffic_signal_status_key)

    @classmethod
    def get_vehicle_location(cls, kvs_client, vehicle_location_key):
        return kvs_client.get(vehicle_location_key)

    @classmethod
    def get_traffic_signal_statuses(cls, kvs_client, traffic_signal_status_keys):
        traffic_signal_statuses = {}
        for traffic_signal_status_key in traffic_signal_status_keys:
            route_code = traffic_signal_status_key.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[1]
            traffic_signal_statuses[route_code] = cls.get_traffic_signal_status(
                kvs_client, traffic_signal_status_key)
        return traffic_signal_statuses

    @classmethod
    def get_vehicle_locations(cls, kvs_client, vehicle_location_keys):
        vehicle_locations = {}
        for vehicle_location_key in vehicle_location_keys:
            vehicle_id = vehicle_location_keys.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[1]
            vehicle_locations[vehicle_id] = cls.get_vehicle_location(kvs_client, vehicle_location_key)
        return vehicle_locations

    @classmethod
    def get_next_vehicle_schedule_start_location_and_pose(cls, vehicle_status, vehicle_schedules, maps_client):
        next_vehicle_schedule_index = Helper.get_next_vehicle_schedule_index(vehicle_status, vehicle_schedules)
        next_route = vehicle_schedules[next_vehicle_schedule_index].route
        next_location = Location.new_location(
            next_route.start_waypoint_id,
            next_route.arrow_codes[0],
            maps_client.waypoint.get_geohash(next_route.start_waypoint_id)
        )
        next_pose = maps_client.waypoint.get_pose(next_route.start_waypoint_id)
        return next_location, next_pose

    @classmethod
    def set_vehicle_location(cls, kvs_client, target_vehicle, vehicle_location, get_key=None):
        return kvs_client.set(
            cls.get_vehicle_location_key(target_vehicle),
            vehicle_location,
            get_key
        )

    @classmethod
    def set_traffic_signal_status(
            cls, kvs_client, target_vehicle, target_traffic_signal, traffic_signal_status, get_key=None):
        return kvs_client.set(
            cls.get_traffic_signal_status_key(target_vehicle, target_traffic_signal),
            traffic_signal_status,
            get_key
        )

    @classmethod
    def current_vehicle_schedule_timeout(cls, vehicle_status, vehicle_schedules):
        return cls.get_current_vehicle_schedule(vehicle_status, vehicle_schedules).period.end < cls.get_current_time()
