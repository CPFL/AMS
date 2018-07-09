#!/usr/bin/env python
# coding: utf-8

from ams.helpers import Topic
from ams.nodes.vehicle import Publisher as VehiclePublisher
from ams.nodes.autoware import CONST, Message


class Publisher(VehiclePublisher):

    VEHICLE = CONST
    VehicleMessage = Message

    @classmethod
    def get_lane_waypoint_array_topic(cls, target_vehicle, target_ros):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_ros,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.BASED_LANE_WAYPOINTS_ARRAY,
        )

    @classmethod
    def get_state_cmd_topic(cls, target_vehicle, target_ros):
        return Topic.get_topic(
            from_target=target_vehicle,
            to_target=target_ros,
            categories=cls.VEHICLE.TOPIC.CATEGORIES.STATE_CMD,
        )

    @classmethod
    def publish_lane_waypoint_array(cls, mqtt_client, target_vehicle, target_ros, lane_waypoint_array):
        topic = cls.get_lane_waypoint_array_topic(target_vehicle, target_ros)
        mqtt_client.publish(topic, lane_waypoint_array)
        print("topic:{}, target_ros{}".format(topic, target_ros))

    @classmethod
    def publish_state_cmd(cls, mqtt_client, target_vehicle, target_ros, state_cmd):
        topic = cls.get_state_cmd_topic(target_vehicle, target_ros)
        mqtt_client.publish(topic, state_cmd)

    # @classmethod
    # def publish_light_color(cls):
    #     monitored_route = cls.get_monitored_route()
    #     if monitored_route is None:
    #         traffic_light = cls.VEHICLE.ROS.TRAFFIC_LIGHT.RED
    #     else:
    #         distance_from_stopline = cls.get_distance_from_stopline(monitored_route)
    #         if distance_from_stopline <= cls.upper_distance_from_stopline:
    #             traffic_light = cls.VEHICLE.ROS.TRAFFIC_LIGHT.RED
    #         else:
    #             traffic_light = cls.VEHICLE.ROS.TRAFFIC_LIGHT.GREEN
    #     header = ROSMessage.Header.get_template()
    #     header.stamp.secs = int(time())
    #     header.stamp.nsecs = int((time() - int(time())) * 1000000000)
    #
    #     payload = Topic.serialize(ROSMessage.LightColor.new_data(
    #         header=header,
    #         traffic_light=traffic_light
    #     ))
    #     cls.publish(cls.__pub_light_color_topic, payload)
