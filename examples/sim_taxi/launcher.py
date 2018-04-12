#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback
import random
from config.env import env


class TaxiSimulation(object):

    MAX_NUM_OF_USERS = 4

    def __init__(self):
        print("launch router")
        self.popen_router = Popen([
            "python", "router.py",
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
            "--path_intersection_json", "./res/intersection.json"
        ])

        sleep(1)

        print("launch taxi fleet")
        self.popen_sim_taxi_fleet = Popen([
            "python", "../node_launchers/sim_taxi_fleet.py",
            "--id", "sim_taxi_fleet_001",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
        ])

        sleep(1)

        print("launch sim taxi 1")
        self.popen_sim_taxi_1 = Popen([
            "python", "../node_launchers/sim_taxi.py",
            "--id", "sim_taxi_001",
            "--name", "taxi_1",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
        ])

        print("launch sim taxi 2")
        self.popen_sim_taxi_2 = Popen([
            "python", "../node_launchers/sim_taxi.py",
            "--id", "sim_taxi_002",
            "--name", "taxi_2",
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

        self.sim_taxi_user_popens = {}

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate taxi fleet")
        self.popen_sim_taxi_fleet.terminate()
        print("terminate sim taxi 1")
        self.popen_sim_taxi_1.terminate()
        print("terminate sim taxi 2")
        self.popen_sim_taxi_2.terminate()
        print("terminate traffic siglnals launcher")
        self.popen_traffic_siglnals_launcher.terminate()
        for sim_taxi_user_popen_id, sim_taxi_user_popen in self.sim_taxi_user_popens.items():
            print("terminate sim taxi user "+str(sim_taxi_user_popen_id))
            sim_taxi_user_popen.terminate()

    def start(self):
        while sleep(3) is None:
            del_popen_ids = []
            for sim_taxi_user_popen_id, sim_taxi_user_popen in self.sim_taxi_user_popens.items():
                if sim_taxi_user_popen.poll() is not None:
                    print("kill taxi user "+str(sim_taxi_user_popen_id))
                    sim_taxi_user_popen.kill()
                    del_popen_ids.append(sim_taxi_user_popen_id)
            for del_popen_id in del_popen_ids:
                self.sim_taxi_user_popens.pop(del_popen_id)

            sim_taxi_user_popen_id = random.randint(0, TaxiSimulation.MAX_NUM_OF_USERS-1)
            if sim_taxi_user_popen_id not in self.sim_taxi_user_popens:
                if 0.0 < random.random():
                    self.sim_taxi_user_popens[sim_taxi_user_popen_id] = Popen([
                        "python", "../node_launchers/sim_taxi_user.py",
                        "--id", "sim_taxi_user_" + str(sim_taxi_user_popen_id),
                        "--name", "user_" + str(sim_taxi_user_popen_id),
                        "--host", env["MQTT_BROKER_HOST"],
                        "--port", env["MQTT_BROKER_PORT"],
                    ])


if __name__ == '__main__':
    taxiSimulation = TaxiSimulation()
    try:
        taxiSimulation.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del taxiSimulation
