#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback
import random
from config.env import env


class TaxiSimulation(object):

    def __init__(self):
        print("launch router")
        self.popen_router = Popen([
            "python", "router.py",
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
            "--path_intersection_json", "./res/intersection.json"
        ])

        sleep(1)

        print("launch sim car 1")
        self.popen_sim_car_1 = Popen([
            "python", "../node_launchers/sim_car.py",
            "--id", "sim_car_001",
            "--name", "car_1",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
        ])

        print("launch sim car 2")
        self.popen_sim_car_2 = Popen([
            "python", "../node_launchers/sim_car.py",
            "--id", "sim_car_002",
            "--name", "car_2",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
        ])

        print("launch traffic signals")
        self.popen_traffic_siglnals_launcher = Popen([
            "python", "../node_launchers/traffic_signals.py",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
            "--path_cycle_json", "../../res/cycle.json",
            "--path_intersection_json", "./res/intersection.json"
        ])

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate sim car 1")
        self.popen_sim_car_1.terminate()
        print("terminate sim car 2")
        self.popen_sim_car_2.terminate()
        print("terminate traffic siglnals launcher")
        self.popen_traffic_siglnals_launcher.terminate()

    def start(self):
        self.popen_sim_car_1.wait()
        self.popen_sim_car_2.wait()


if __name__ == '__main__':
    taxiSimulation = TaxiSimulation()
    try:
        taxiSimulation.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del taxiSimulation
