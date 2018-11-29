#!/usr/bin/env python
# coding: utf-8

from subprocess import Popen
from time import sleep
import traceback

from ams import logger


class Launcher(object):

    def __init__(self):
        print("launch viewer")
        self.popen_viewer = Popen(["python", "viewer.py"])

        sleep(1)

        print("launch vehicle")
        command = "python ../node_launcher/vehicle.py -KCT redis -PSCT paho -TD sim -IFP ./initials/vehicle.json -SMP ./state_machines/vehicle.json"
        self.popen_vehicle = Popen(command.split(" "))

        print("launch vehicle")
        command = "python ../node_launcher/vehicle.py -KCT redis -PSCT paho -TD sim -IFP ./initials/vehicle.json -SMP ./state_machines/vehicle.json"
        self.popen_vehicle1 = Popen(command.split(" "))

        print("launch milee autoware")
        command = "python ../node_launcher/autoware.py -KCT redis -PSCT paho -TD sim -IFP ./initials/autoware.json -SMP ./state_machines/autoware.json"
        self.popen_milee_autoware = Popen(command.split(" "))

        print("launch milee autoware interface")
        command = "python ../node_launcher/autoware_interface.py -KCT redis -PSCT paho -TD sim -IFP ./initials/autoware_interface.json"
        self.popen_milee_autoware_interface = Popen(command.split(" "))

        print("launch traffic_signal")
        command = "python ../node_launcher/traffic_signal.py -KCT redis -PSCT paho -TD sim -IFP ./initials/traffic_signal.json -SMP ./state_machines/traffic_signal.json"
        self.popen_traffic_signal = Popen(command.split(" "))

        # print("launch shutter")
        # command = "python ../node_launcher/shutter.py -PSCT paho -TD sim -IFP ./initials/shutter.json"
        # self.popen_shutter = Popen(command.split(" "))
        #
        # print("launch light")
        # command = "python ../node_launcher/light.py -PSCT paho -TD sim -IFP ./initials/light.json"
        # self.popen_light = Popen(command.split(" "))

    def __del__(self):
        self.popen_viewer.terminate()
        self.popen_vehicle.terminate()
        self.popen_vehicle1.terminate()
        self.popen_milee_autoware.terminate()
        self.popen_milee_autoware_interface.terminate()
        self.popen_traffic_signal.terminate()
        # self.popen_shutter.terminate()
        # self.popen_light.terminate()

    def start(self):
        self.popen_viewer.wait()


if __name__ == '__main__':
    launcher = Launcher()
    try:
        launcher.start()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(logger.pformat(e))
    finally:
        del launcher
