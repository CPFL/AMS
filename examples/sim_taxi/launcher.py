#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback
from config.env import env


class TaxiSimulation(object):
    def __init__(self):
        print("launch router")
        self.popen_router = Popen(["python", "router.py"])

        sleep(1)

        print("launch fleet manager")
        self.popen_fleet_manager = Popen(["python", "../node_launchers/fleet_manager.py"])

        sleep(1)

        print("launch sim taxi")
        self.popen_sim_taxi = Popen(["python", "../node_launchers/sim_taxi.py", "taxi"])

        print("launch traffic signals")
        self.popen_traffic_siglnals_launcher = Popen([
            "python", "../node_launchers/traffic_signals.py", env["MQTT_BROKER_HOST"], env["MQTT_BROKER_PORT"]])

        self.popen_user = None

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate fleet manager")
        self.popen_fleet_manager.terminate()
        print("terminate sim taxi")
        self.popen_sim_taxi.terminate()
        print("terminate traffic siglnals launcher")
        self.popen_traffic_siglnals_launcher.terminate()
        if self.popen_user is not None:
            print("terminate user")
            self.popen_user.terminate()

    def start(self):
        while sleep(3) is None:
            if self.popen_user is None:
                self.popen_user = Popen(["python", "../node_launchers/user.py", "user"])
                self.popen_user.wait()
                self.popen_user.kill()
                self.popen_user = None


if __name__ == '__main__':
    taxiSimulation = TaxiSimulation()
    try:
        taxiSimulation.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del taxiSimulation
