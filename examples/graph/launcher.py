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

        print("launch graph monitor")
        self.popen_graph_monitor = Popen([
            "python", "./graph_monitor.py",
        ])

        sleep(1)

    def __del__(self):
        print("terminate router")
        self.popen_router.terminate()
        print("terminate graph monitor")
        self.popen_graph_monitor.terminate()

    def start(self):
        self.popen_graph_monitor.wait()

if __name__ == '__main__':
    taxiSimulation = TaxiSimulation()
    try:
        taxiSimulation.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del taxiSimulation
