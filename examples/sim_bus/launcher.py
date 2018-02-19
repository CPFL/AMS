#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback
import random
from config.env import env


class BusServiceLauncher(object):

    MAX_NUM_OF_USERS = 2

    def __init__(self):
        print("launch router")
        self.popen_router = Popen(
            ["python", "router.py"]
        )

        sleep(1)

        print("launch bus fleet")
        self.popen_bus_fleet = Popen([
            "python", "../node_launchers/bus_fleet.py",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
            "--name", "sim_bus_fleet",
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
            "--path_spot_json", "../../res/spot.json",
            "--path_bus_schedule_json", "../../res/bus_schedule.json"
        ])

        sleep(1)

        print("launch sim bus 1")
        self.popen_sim_bus_1 = Popen([
            "python", "../node_launchers/sim_bus.py",
            "--name", "bus_1",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
            "--path_waypoint_json", "../../res/waypoint.json",
            "--path_arrow_json", "../../res/arrow.json",
            "--path_intersection_json", "../../res/intersection.json",
        ])

        # print("launch sim bus 2")
        # self.popen_sim_bus_2 = Popen([
        #     "python", "../node_launchers/sim_bus.py",
        #     "--name", "bus_2",
        #     "--host", env["MQTT_BROKER_HOST"],
        #     "--port", env["MQTT_BROKER_PORT"],
        #     "--path_waypoint_json", "../../res/waypoint.json",
        #     "--path_arrow_json", "../../res/arrow.json",
        #     "--path_intersection_json", "../../res/intersection.json",
        # ])

        print("launch traffic signals")
        self.popen_traffic_siglnals_launcher = Popen([
            "python", "../node_launchers/traffic_signals.py",
            "--host", env["MQTT_BROKER_HOST"],
            "--port", env["MQTT_BROKER_PORT"],
            "--path_cycle_json", "../../res/cycle.json",
            "--path_intersection_json", "../../res/intersection.json"
        ])

        self.bus_user_popens = {}

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate bus fleet")
        self.popen_bus_fleet.terminate()
        print("terminate sim bus 1")
        self.popen_sim_bus_1.terminate()
        # print("terminate sim bus 2")
        # self.popen_sim_bus_2.terminate()

        print("terminate traffic siglnals launcher")
        self.popen_traffic_siglnals_launcher.terminate()

        for bus_user_popen_id, bus_user_popen in self.bus_user_popens.items():
            print("terminate bus user "+str(bus_user_popen_id))
            bus_user_popen.terminate()

    def start(self):
        while sleep(3) is None:
            pass
            del_popen_ids = []
            for bus_user_popen_id, bus_user_popen in self.bus_user_popens.items():
                if bus_user_popen.poll() is not None:
                    print("kill bus user "+str(bus_user_popen_id))
                    bus_user_popen.kill()
                    del_popen_ids.append(bus_user_popen_id)
            for del_popen_id in del_popen_ids:
                self.bus_user_popens.pop(del_popen_id)

            bus_user_popen_id = random.randint(0, BusServiceLauncher.MAX_NUM_OF_USERS-1)
            if bus_user_popen_id not in self.bus_user_popens:
                if 0.0 < random.random():
                    self.bus_user_popens[bus_user_popen_id] = Popen([
                        "python", "../node_launchers/bus_user.py",
                        "--name", "user_"+str(bus_user_popen_id),
                        "--host", env["MQTT_BROKER_HOST"],
                        "--port", env["MQTT_BROKER_PORT"],
                        "--path_spot_json", "../../res/spot.json",
                    ])


if __name__ == '__main__':
    busService = BusServiceLauncher()
    try:
        busService.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del busService
