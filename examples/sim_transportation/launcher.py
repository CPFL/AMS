#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback
import random
from config.env import env


class TransportationsLauncher(object):

    NUM_OF_TAXI = 2
    NUM_OF_BUS = 1
    LAUNCH_TRAFFIC_SIGNALS = False
    MAX_NUM_OF_USERS = 3

    def __init__(self):
        print("launch router")
        self.popen_router = Popen([
            "python", "router.py",
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
            "--path_intersection_json", "./res/intersection.json"
        ])

        sleep(1)

        if 0 < TransportationsLauncher.NUM_OF_TAXI:
            print("launch taxi fleet")
            self.popen_sim_taxi_fleet = Popen([
                "python", "../node_launchers/sim_taxi_fleet.py",
                "--id", "sim_taxi_fleet_001",
                "--host", env["MQTT_BROKER_HOST"],
                "--port", env["MQTT_BROKER_PORT"],
            ])

            sleep(1)

            self.popen_sim_taxi = {}
            for i in range(TransportationsLauncher.NUM_OF_TAXI):
                print("launch sim taxi " + str(i))
                self.popen_sim_taxi[i] = Popen([
                    "python", "../node_launchers/sim_taxi.py",
                    "--id", "sim_taxi_" + str(i),
                    "--name", "taxi_" + str(i),
                    "--host", env["MQTT_BROKER_HOST"],
                    "--port", env["MQTT_BROKER_PORT"],
                ])

        if 0 < TransportationsLauncher.NUM_OF_BUS:
            print("launch bus fleet")
            self.popen_sim_bus_fleet = Popen([
                "python", "../node_launchers/sim_bus_fleet.py",
                "--host", env["MQTT_BROKER_HOST"],
                "--port", env["MQTT_BROKER_PORT"],
                "--id", "sim_bus_fleet_001",
                "--path_waypoint_json", "../../res/waypoint.json",
                "--path_arrow_json", "../../res/arrow.json",
                "--path_spot_json", "../../res/spot.json",
                "--path_bus_schedule_json", "../../res/bus_schedule.json"
            ])

            sleep(1)

            self.popen_sim_bus = {}
            for i in range(TransportationsLauncher.NUM_OF_BUS):
                print("launch sim bus " + str(i))
                self.popen_sim_bus[i] = Popen([
                    "python", "../node_launchers/sim_bus.py",
                    "--id", "sim_bus_" + str(i),
                    "--name", "sim_bus_" + str(i),
                    "--host", env["MQTT_BROKER_HOST"],
                    "--port", env["MQTT_BROKER_PORT"],
                    "--path_waypoint_json", "../../res/waypoint.json",
                    "--path_arrow_json", "../../res/arrow.json",
                    "--path_intersection_json", "../../res/intersection.json",
                ])

        if TransportationsLauncher.LAUNCH_TRAFFIC_SIGNALS:
            print("launch traffic signals")
            self.popen_traffic_siglnals_launcher = Popen([
                "python", "../node_launchers/traffic_signals.py",
                "--host", env["MQTT_BROKER_HOST"],
                "--port", env["MQTT_BROKER_PORT"],
                "--path_cycle_json", "../../res/cycle.json",
                "--path_intersection_json", "../../res/intersection.json"
            ])

        self.sim_taxi_user_popens = {}
        self.sim_bus_user_popens = {}

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()

        if 0 < TransportationsLauncher.NUM_OF_TAXI:
            print("terminate taxi fleet")
            self.popen_sim_taxi_fleet.terminate()
            print("terminate sim taxi")
            for i in self.popen_sim_taxi:
                self.popen_sim_taxi[i].terminate()

        if 0 < TransportationsLauncher.NUM_OF_BUS:
            print("terminate bus fleet")
            self.popen_sim_bus_fleet.terminate()
            print("terminate sim bus 1")
            for i in self.popen_sim_bus:
                self.popen_sim_bus[i].terminate()

        if TransportationsLauncher.LAUNCH_TRAFFIC_SIGNALS:
            print("terminate traffic siglnals launcher")
            self.popen_traffic_siglnals_launcher.terminate()

        for sim_bus_user_popen_id, sim_bus_user_popen in self.sim_bus_user_popens.items():
            print("terminate bus user "+str(sim_bus_user_popen_id))
            sim_bus_user_popen.terminate()
        for sim_taxi_user_popen_id, sim_taxi_user_popen in self.sim_taxi_user_popens.items():
            print("terminate sim taxi user "+str(sim_taxi_user_popen_id))
            sim_taxi_user_popen.terminate()

    def start(self):
        while sleep(3) is None:

            if 0 < TransportationsLauncher.NUM_OF_TAXI:
                del_popen_ids = []
                for sim_taxi_user_popen_id, sim_taxi_user_popen in self.sim_taxi_user_popens.items():
                    if sim_taxi_user_popen.poll() is not None:
                        print("kill taxi user "+str(sim_taxi_user_popen_id))
                        sim_taxi_user_popen.kill()
                        del_popen_ids.append(sim_taxi_user_popen_id)
                for del_popen_id in del_popen_ids:
                    self.sim_taxi_user_popens.pop(del_popen_id)

                sim_taxi_user_popen_id = random.randint(0, TransportationsLauncher.MAX_NUM_OF_USERS-1)
                if sim_taxi_user_popen_id not in self.sim_taxi_user_popens:
                    if 0.0 < random.random():
                        self.sim_taxi_user_popens[sim_taxi_user_popen_id] = Popen([
                            "python", "../node_launchers/sim_taxi_user.py",
                            "--id", "sim_taxi_user_" + str(sim_taxi_user_popen_id),
                            "--name", "sim_taxi_user_" + str(sim_taxi_user_popen_id),
                            "--host", env["MQTT_BROKER_HOST"],
                            "--port", env["MQTT_BROKER_PORT"],
                        ])

            if 0 < TransportationsLauncher.NUM_OF_BUS:
                del_popen_ids = []
                for sim_bus_user_popen_id, sim_bus_user_popen in self.sim_bus_user_popens.items():
                    if sim_bus_user_popen.poll() is not None:
                        print("kill bus user "+str(sim_bus_user_popen_id))
                        sim_bus_user_popen.kill()
                        del_popen_ids.append(sim_bus_user_popen_id)
                for del_popen_id in del_popen_ids:
                    self.sim_bus_user_popens.pop(del_popen_id)

                sim_bus_user_popen_id = random.randint(0, TransportationsLauncher.MAX_NUM_OF_USERS-1)
                if sim_bus_user_popen_id not in self.sim_bus_user_popens:
                    if 0.8 < random.random():
                        self.sim_bus_user_popens[sim_bus_user_popen_id] = Popen([
                            "python", "../node_launchers/sim_bus_user.py",
                            "--id", "sim_bus_user_" + str(sim_bus_user_popen_id),
                            "--name", "sim_bus_user_"+str(sim_bus_user_popen_id),
                            "--host", env["MQTT_BROKER_HOST"],
                            "--port", env["MQTT_BROKER_PORT"],
                            "--path_spot_json", "../../res/spot.json",
                        ])


if __name__ == '__main__':
    transportations = TransportationsLauncher()
    try:
        transportations.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del transportations
