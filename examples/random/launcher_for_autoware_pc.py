#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback


class Launcher(object):

    def __init__(self):
        print("launch viewer")
        self.popen_viewer = Popen(["python", "viewer.py"])

        sleep(1)

        print("launch vehicle")
        command = "python ../node_launcher/vehicle.py -PSCT paho -TD sim -IFP ./initials/vehicle.json -SMP ./state_machines/vehicle.json"
        self.popen_vehicle = Popen(command.split(" "))

        sleep(1)

        print("launch autoware_interface")
        command = "python ../node_launcher/autoware_interface.py -PSCT paho -TD sim -IFP ./initials/autoware_interface.json"
        self.popen_autoware_interface = Popen(command.split(" "))

    def __del__(self):
        self.popen_viewer.terminate()
        self.popen_vehicle.terminate()
        self.popen_autoware_interface.terminate()

    def start(self):
        self.popen_viewer.wait()


if __name__ == '__main__':
    lanucher = Launcher()
    try:
        lanucher.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del lanucher
