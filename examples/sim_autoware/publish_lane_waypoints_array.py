#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

import paho.mqtt.client

from ams import MapsClient, get_ams_mqtt_client_class
from ams.nodes import Autoware


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-WFP", "--waypoint_file_path", type=str, required=True, help="waypoint.json file path")
    parser.add_argument("-AFP", "--arrow_file_path", type=str, required=True, help="array.json file path")
    parser.add_argument("-RC", "--route_code", type=str, required=True, help="route_code")
    parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt host")
    parser.add_argument("-MP", "--mqtt_port", type=str, default=1883, help="mqtt port")
    parser.add_argument("-MT", "--mqtt_topic", type=str, required=True, help="mqtt topic")
    args = parser.parse_args()

    maps_client = MapsClient()
    maps_client.load_waypoint_json_file(args.waypoint_file_path)
    maps_client.load_arrow_json_file(args.arrow_file_path)

    route = maps_client.route.decode_route_code(args.route_code)
    route_detail = maps_client.route.get_detail(route)
    lane_waypoint_array = Autoware.Helper.get_lane_waypoint_array_from_route_detail(route_detail)

    MQTTClient = get_ams_mqtt_client_class(paho.mqtt.client)
    mqtt_client = MQTTClient()
    mqtt_client.set_args_of_Client()
    mqtt_client.set_args_of_connect(host=args.mqtt_host, port=args.mqtt_port)
    mqtt_client.connect()
    mqtt_client.publish(args.mqtt_topic, lane_waypoint_array)
