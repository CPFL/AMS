#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback
from argparse import ArgumentParser

from config.env import env


parser = ArgumentParser()
parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
parser.add_argument("-N", "--name", type=str, default="a1", help="name")
args = parser.parse_args()


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
            "--id", args.id,
            "--name", args.name,
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
        ])

        sleep(1)

        # print("launch traffic signals")
        # self.popen_traffic_signals = Popen([
        #     "python", "../node_launchers/traffic_signals.py",
        #     "--host", env["MQTT_BROKER_HOST"],
        #     "--port", env["MQTT_BROKER_PORT"],
        #     "--path_intersection_json", "./res/intersection.json"
        # ])

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate autoware 1")
        self.popen_autoware_1.terminate()
        # print("terminate TrafficSiglnals")
        # self.popen_traffic_signals.terminate()

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
