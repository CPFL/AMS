#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback
from config.env import env


class AutowareSimulation(object):
    def __init__(self):
        print("launch router")
        self.popen_router = Popen([
            "python", "router.py",
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
        ])

        sleep(1)

        print("launch Autoware")
        self.popen_autoware_1 = Popen([
            "python", "../node_launchers/autoware.py",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
            "--id", "a1",
            "--name", "a1",
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
        ])

        sleep(1)

        print("launch traffic signals")
        self.popen_traffic_signals = Popen([
            "python", "../node_launchers/traffic_signals.py",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
            "--path_intersection_json", "./res/intersection.json"
        ])

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate autoware 1")
        self.popen_autoware_1.terminate()
        print("terminate TrafficSiglnals")
        self.popen_traffic_signals.terminate()

    @staticmethod
    def start():
        while sleep(10) is None:
            pass


if __name__ == '__main__':
    autoware_simulation = AutowareSimulation()
    try:
        autoware_simulation.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del autoware_simulation
