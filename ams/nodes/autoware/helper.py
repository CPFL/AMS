#!/usr/bin/env python
# coding: utf-8

from time import time
from math import modf

from ams import logger, MapsClient
from ams.helpers import Target, Location
from ams.structures import KVS_CLIENT, Pose
from ams.nodes.vehicle import Helper as VehicleHelper
from ams.nodes.traffic_signal import CONST as TRAFFIC_SIGNAL
from ams.nodes.autoware import CONST, Structure


class Helper(VehicleHelper):

    VEHICLE = CONST
    VehicleStructure = Structure

    @classmethod
    def get_traffic_signal_status_key(cls, target_vehicle, target_traffic_signal):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_vehicle), Target.get_code(target_traffic_signal), "status"])

    @classmethod
    def get_all_traffic_signal_status_keys(cls, kvs_client, target_vehicle):
        return kvs_client.keys(KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_vehicle),
            Target.get_code(Target.new_target(group=TRAFFIC_SIGNAL.NODE_NAME)),
            "status"
        ]))

    @classmethod
    def get_traffic_signal_status(cls, kvs_client, traffic_signal_status_key):
        return kvs_client.get(traffic_signal_status_key)

    @classmethod
    def get_traffic_signal_statuses(cls, kvs_client, traffic_signal_status_keys):
        traffic_signal_statuses = {}
        for traffic_signal_status_key in traffic_signal_status_keys:
            route_code = traffic_signal_status_key.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[1]
            traffic_signal_statuses[route_code] = cls.get_traffic_signal_status(
                kvs_client, traffic_signal_status_key)
        return traffic_signal_statuses

    @classmethod
    def set_traffic_signal_status(
            cls, kvs_client, target_vehicle, target_traffic_signal, traffic_signal_status, get_key=None):
        return kvs_client.set(
            cls.get_traffic_signal_status_key(target_vehicle, target_traffic_signal),
            traffic_signal_status,
            get_key
        )

    @classmethod
    def get_pose_from_current_pose(cls, current_pose):
        return Pose.new_data(
            position=Pose.Position.new_data(**current_pose.pose.position),
            orientation=Pose.Orientation.new_data(
                quaternion=Pose.Orientation.Quaternion.new_data(**current_pose.pose.orientation),
                rpy=None
            )
        )

    @classmethod
    def get_location_from_closest_waypoint(cls, maps_client, closest_waypoint, vehicle_status):
        route = maps_client.route.decode_route_code(vehicle_status.route_code)
        locations = maps_client.route.get_locations(route)
        return locations[closest_waypoint.data]

    @classmethod
    def get_pose_from_location(cls, maps_client, location):
        return maps_client.arrow.get_pose(location.arrow_code, location.waypoint_id)

    @classmethod
    def get_lane_waypoint_array_from_locations(cls, locations, maps_client):
        if 0 == len(locations):
            return None

        logger.info(locations)

        header = cls.VehicleStructure.ROSMessage.Header.get_template()
        nsec, sec = modf(time())
        header.stamp.secs = int(sec)
        header.stamp.nsecs = int(nsec*(10**9))

        lane_array = cls.VehicleStructure.ROSMessage.LaneArray.get_template()
        lane_array.lanes[0].header = header
        lane_array.lanes[0].waypoints = []

        for location in locations:
            pose = maps_client.waypoint.get_pose(location.waypoint_id)
            waypoint = cls.VehicleStructure.ROSMessage.LaneArray.Lane.Waypoint.get_template()
            waypoint.pose.header = header
            waypoint.pose.pose.position.x = pose.position.x
            waypoint.pose.pose.position.y = pose.position.y
            waypoint.pose.pose.position.z = pose.position.z
            waypoint.pose.pose.orientation.z = pose.orientation.quaternion.z
            waypoint.pose.pose.orientation.w = pose.orientation.quaternion.w

            waypoint.twist.header = header
            waypoint.twist.twist.linear.x = 0.2 * maps_client.waypoint.get_speed_limit(location.waypoint_id)

            lane_array.lanes[0].waypoints.append(waypoint)

        return lane_array

    @classmethod
    def get_next_schedule_lane_waypoint_array(cls, vehicle_status, vehicle_schedules, maps_client):
        next_vehicle_schedule_index = cls.get_next_vehicle_schedule_index(vehicle_status, vehicle_schedules)
        route = vehicle_schedules[next_vehicle_schedule_index].route
        locations = maps_client.route.get_locations(route)
        return cls.get_lane_waypoint_array_from_locations(locations, maps_client)

    @classmethod
    def get_state_cmd_from_data(cls, data):
        return cls.VehicleStructure.ROSMessage.StateCMD.new_data(data=data)

    @classmethod
    def get_vehicle_route_code(cls, vehicle_status, vehicle_schedules):
        next_vehicle_schedule_index = cls.get_next_vehicle_schedule_index(vehicle_status, vehicle_schedules)
        route = vehicle_schedules[next_vehicle_schedule_index].route
        return MapsClient.Route.encode_route(route)

    @classmethod
    def update_and_set_vehicle_pose(cls, kvs_client, target_vehicle, vehicle_status, maps_client):
        # on closest_waypoint_ros_message
        if vehicle_status.route_code is not None and \
                vehicle_status.decision_maker_state != cls.VEHICLE.ROS.DECISION_MAKER_STATE.WAIT_ORDER:
            vehicle_status.location = cls.get_location_from_closest_waypoint(
                maps_client, vehicle_status.closest_waypoint, vehicle_status)
            vehicle_status.pose = cls.get_pose_from_location(maps_client, vehicle_status.location)

            cls.set_vehicle_status(kvs_client, target_vehicle, vehicle_status)
        else:
            # on current_pose_ros_message
            if vehicle_status.location is None and vehicle_status.current_pose is not None:
                # print("Autoware.Subscriber.on_current_pose_ros_message", vehicle_status)
                vehicle_status.location = maps_client.map_match.get_matched_location_on_arrows(
                    cls.get_pose_from_current_pose(vehicle_status.current_pose))

                if vehicle_status.location is not None:
                    vehicle_status.pose = cls.get_pose_from_location(maps_client, vehicle_status.location)

                    cls.set_vehicle_status(kvs_client, target_vehicle, vehicle_status)

    # @classmethod
    # def update_traffic_signal_status_subscribers(cls, vehicle_status):
    #     traffic_signal_arrow_code = cls.get_target_traffic_signal_arrow_codes_on_route(vehicle_status.route_code)
    #     # todo
    #     return
