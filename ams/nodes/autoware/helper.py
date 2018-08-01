#!/usr/bin/env python
# coding: utf-8

from time import time
from math import modf

from ams import logger, MapsClient
from ams.helpers import Target
from ams.structures import KVS_CLIENT, Pose
from ams.nodes.vehicle import Helper as VehicleHelper
from ams.nodes.traffic_signal import CONST as TRAFFIC_SIGNAL
from ams.nodes.autoware import CONST, Structure


class Helper(VehicleHelper):

    VEHICLE = CONST
    Structure = Structure

    @classmethod
    def get_current_pose_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles[cls.VEHICLE.ROLE_NAME]),
                Target.get_code(target_roles[cls.VEHICLE.ROS.ROLE_NAME])
            ] + cls.VEHICLE.TOPIC.CATEGORIES.CURRENT_POSE)

    @classmethod
    def get_closest_waypoint_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles[cls.VEHICLE.ROLE_NAME]),
                Target.get_code(target_roles[cls.VEHICLE.ROS.ROLE_NAME])
            ] + cls.VEHICLE.TOPIC.CATEGORIES.CLOSEST_WAYPOINT)

    @classmethod
    def get_traffic_signal_status_key(cls, target_roles):
        return KVS_CLIENT.KEY_PATTERN_DELIMITER.join(
            [
                Target.get_code(target_roles[cls.VEHICLE.ROLE_NAME]),
                Target.get_code(target_roles["traffic_signal"]), "status"
            ])

    @classmethod
    def get_all_traffic_signal_status_keys(cls, clients, target_roles):
        return clients["kvs"].keys(KVS_CLIENT.KEY_PATTERN_DELIMITER.join([
            Target.get_code(target_roles["vehicle"]),
            Target.get_code(Target.new_target(group=TRAFFIC_SIGNAL.NODE_NAME)),
            "status"
        ]))

    @classmethod
    def get_traffic_signal_status(cls, clients, traffic_signal_status_key):
        return clients["kvs"].get(traffic_signal_status_key)

    @classmethod
    def get_traffic_signal_statuses(cls, clients, traffic_signal_status_keys):
        traffic_signal_statuses = {}
        for traffic_signal_status_key in traffic_signal_status_keys:
            route_code = traffic_signal_status_key.split(KVS_CLIENT.KEY_PATTERN_DELIMITER)[1]
            traffic_signal_statuses[route_code] = cls.get_traffic_signal_status(
                clients["kvs"], traffic_signal_status_key)
        return traffic_signal_statuses

    @classmethod
    def get_current_pose(cls, clients, target_roles):
        key = cls.get_current_pose_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.ROSMessage.CurrentPose.new_data(**value)
        return value

    @classmethod
    def get_closest_waypoint(cls, clients, target_roles):
        key = cls.get_closest_waypoint_key(target_roles)
        value = clients["kvs"].get(key)
        if value.__class__.__name__ == "dict":
            value = cls.Structure.ROSMessage.ClosestWaypoint.new_data(**value)
        return value

    @classmethod
    def set_traffic_signal_status(cls, clients, target_roles, traffic_signal_status, get_key=None):
        return clients["kvs"].set(cls.get_traffic_signal_status_key(clients), traffic_signal_status, get_key)

    @classmethod
    def set_current_pose(cls, clients, target_roles, current_pose):
        key = cls.get_current_pose_key(target_roles)
        return clients["kvs"].set(key, current_pose)

    @classmethod
    def set_closest_waypoint(cls, clients, target_roles, closest_waypoint):
        key = cls.get_closest_waypoint_key(target_roles)
        return clients["kvs"].set(key, closest_waypoint)

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
    def get_location_from_closest_waypoint(cls, clients, closest_waypoint, vehicle_status):
        route = clients["maps"].route.decode_route_code(vehicle_status.route_code)
        locations = clients["maps"].route.get_locations(route)
        return locations[closest_waypoint.data]

    @classmethod
    def get_pose_from_location(cls, clients, location):
        return clients["maps"].arrow.get_pose(location.arrow_code, location.waypoint_id)

    @classmethod
    def get_lane_waypoint_array_from_locations(cls, clients, locations):
        if 0 == len(locations):
            return None

        logger.info(locations)

        header = cls.Structure.ROSMessage.Header.get_template()
        nsec, sec = modf(time())
        header.stamp.secs = int(sec)
        header.stamp.nsecs = int(nsec*(10**9))

        lane_array = cls.Structure.ROSMessage.LaneArray.get_template()
        lane_array.lanes[0].header = header
        lane_array.lanes[0].waypoints = []

        for location in locations:
            pose = clients["maps"].waypoint.get_pose(location.waypoint_id)
            waypoint = cls.Structure.ROSMessage.LaneArray.Lane.Waypoint.get_template()
            waypoint.pose.header = header
            waypoint.pose.pose.position.x = pose.position.x
            waypoint.pose.pose.position.y = pose.position.y
            waypoint.pose.pose.position.z = pose.position.z
            waypoint.pose.pose.orientation.z = pose.orientation.quaternion.z
            waypoint.pose.pose.orientation.w = pose.orientation.quaternion.w

            waypoint.twist.header = header
            waypoint.twist.twist.linear.x = 0.2 * clients["maps"].waypoint.get_speed_limit(location.waypoint_id)

            lane_array.lanes[0].waypoints.append(waypoint)

        return lane_array

    @classmethod
    def get_next_schedule_lane_waypoint_array(cls, clients, vehicle_status, vehicle_schedules):
        next_vehicle_schedule_index = cls.get_next_vehicle_schedule_index(vehicle_status, vehicle_schedules)
        route = vehicle_schedules[next_vehicle_schedule_index].route
        locations = clients["maps"].route.get_locations(route)
        return cls.get_lane_waypoint_array_from_locations(clients, locations)

    @classmethod
    def get_state_cmd_from_data(cls, data):
        return cls.Structure.ROSMessage.StateCMD.new_data(data=data)

    @classmethod
    def get_vehicle_route_code(cls, vehicle_status, vehicle_schedules):
        next_vehicle_schedule_index = cls.get_next_vehicle_schedule_index(vehicle_status, vehicle_schedules)
        route = vehicle_schedules[next_vehicle_schedule_index].route
        return MapsClient.Route.encode_route(route)

    @classmethod
    def update_and_set_vehicle_pose(cls, clients, target_roles, vehicle_status):
        # on closest_waypoint_ros_message
        if vehicle_status.route_code is not None and \
                vehicle_status.decision_maker_state != cls.VEHICLE.ROS.DECISION_MAKER_STATE.WAIT_ORDER:
            vehicle_status.location = cls.get_location_from_closest_waypoint(
                clients, vehicle_status.closest_waypoint, vehicle_status)
            vehicle_status.pose = cls.get_pose_from_location(clients, vehicle_status.location)

            cls.set_vehicle_status(clients, target_roles, vehicle_status)
        else:
            # on current_pose_ros_message
            if vehicle_status.location is None and vehicle_status.current_pose is not None:
                vehicle_status.location = clients["maps"].map_match.get_matched_location_on_arrows(
                    cls.get_pose_from_current_pose(vehicle_status.current_pose))

                if vehicle_status.location is not None:
                    vehicle_status.pose = cls.get_pose_from_location(clients, vehicle_status.location)

                    cls.set_vehicle_status(clients, target_roles, vehicle_status)

    # @classmethod
    # def update_traffic_signal_status_subscribers(cls, vehicle_status):
    #     traffic_signal_arrow_code = cls.get_target_traffic_signal_arrow_codes_on_route(vehicle_status.route_code)
    #     # todo
    #     return
