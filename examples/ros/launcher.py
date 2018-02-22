#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen

from config.env import env


if __name__ == '__main__':

    popen_lane_array_publisher = Popen([
        "python", "lane_array_publisher.py",
        "--name", "a1",
        "--host", env["MQTT_BROKER_HOST"],
        "--port", env["MQTT_BROKER_PORT"],
    ])
    popen_closest_waypoint_subscriber = Popen([
        "python", "closest_waypoint_subscriber.py",
        "--name", "a1",
        "--host", env["MQTT_BROKER_HOST"],
        "--port", env["MQTT_BROKER_PORT"],
        "--period", "1.0"
    ])
    popen_light_color_managed_publisher = Popen([
        "python", "light_color_managed_publisher.py",
        "--name", "a1",
        "--host", env["MQTT_BROKER_HOST"],
        "--port", env["MQTT_BROKER_PORT"],
    ])

    try:
        popen_lane_array_publisher.wait()

    except KeyboardInterrupt:
        popen_lane_array_publisher.terminate()
        popen_closest_waypoint_subscriber.terminate()
        popen_light_color_managed_publisher.terminate()
