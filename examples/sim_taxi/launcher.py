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
        self.popen_router = Popen(["python", "router.py"])

        sleep(1)

        print("launch taxi fleet")
        self.popen_taxi_fleet = Popen([
            "python", "../node_launchers/taxi_fleet.py",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
        ])

        sleep(1)

        print("launch sim taxi 1")
        self.popen_sim_taxi_1 = Popen([
            "python", "../node_launchers/sim_taxi.py",
            "--name", "taxi_1",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
        ])

        # print("launch sim taxi 2")
        # self.popen_sim_taxi_2 = Popen(["python", "../node_launchers/sim_taxi.py", env["MQTT_BROKER_HOST"], env["MQTT_BROKER_PORT"], "taxi_2"])

        print("launch traffic signals")
        self.popen_traffic_siglnals_launcher = Popen([
            "python", "../node_launchers/traffic_signals.py",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
            "--path_cycle_json", "../../res/cycle.json"
        ])

        self.popen_taxi_user_1 = None
        self.popen_taxi_user_2 = None

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate taxi fleet")
        self.popen_taxi_fleet.terminate()
        print("terminate sim taxi 1")
        self.popen_sim_taxi_1.terminate()
        # print("terminate sim taxi 2")
        # self.popen_sim_taxi_2.terminate()
        print("terminate traffic siglnals launcher")
        self.popen_traffic_siglnals_launcher.terminate()
        if self.popen_taxi_user_1 is not None:
            print("terminate taxi user 1")
            self.popen_taxi_user_1.terminate()
        if self.popen_taxi_user_2 is not None:
            print("terminate taxi user 2")
            self.popen_taxi_user_2.terminate()

    def start(self):
        while sleep(3) is None:
            if self.popen_taxi_user_1 is None:
                if 0.6 < random.random():
                    self.popen_taxi_user_1 = Popen([
                        "python", "../node_launchers/taxi_user.py",
                        "--name", "user_1",
                        "--host", env["MQTT_BROKER_HOST"],
                        "--port", env["MQTT_BROKER_PORT"],
                    ])
                    self.popen_taxi_user_1.wait()
                    self.popen_taxi_user_1.kill()
                    self.popen_taxi_user_1 = None

            # if self.popen_taxi_user_2 is None:
            #     if 0.6 < random.random():
            #         self.popen_taxi_user_2 = Popen(["python", "../node_launchers/user.py", "user_2"])
            #         self.popen_taxi_user_2.wait()
            #         self.popen_taxi_user_2.kill()
            #         self.popen_taxi_user_2 = None


if __name__ == '__main__':
    taxiSimulation = TaxiSimulation()
    try:
        taxiSimulation.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del taxiSimulation
