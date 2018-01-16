#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
import sys


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("input autoware_taxi name.")
    try:
        popen_lane_array_publisher = Popen(["python", "lane_array_publisher.py", sys.argv[1]])
        popen_closest_waypoint_subscriber = Popen(["python", "closest_waypoint_subscriber.py", sys.argv[1]])
        popen_light_color_managed_publisher = Popen(["python", "light_color_managed_publisher.py", sys.argv[1]])

        popen_lane_array_publisher.wait()
    except KeyboardInterrupt:
        popen_lane_array_publisher.terminate()
        popen_closest_waypoint_subscriber.terminate()
        popen_light_color_managed_publisher.terminate()

