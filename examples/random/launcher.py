#!/usr/bin/env python
# coding: utf-8

import signal
from subprocess import Popen
from time import sleep

from setproctitle import setproctitle

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

        print("launch user")
        command = "python ../node_launcher/user.py -KCT redis -PSCT paho -TD sim -IFP ./initials/user.json -SMP ./state_machines/user.json"
        self.popen_user = Popen(command.split(" "))

        # print("launch shutter")
        # command = "python ../node_launcher/shutter.py -PSCT paho -TD sim -IFP ./initials/shutter.json"
        # self.popen_shutter = Popen(command.split(" "))
        #
        # print("launch light")
        # command = "python ../node_launcher/light.py -PSCT paho -TD sim -IFP ./initials/light.json"
        # self.popen_light = Popen(command.split(" "))

    def __delete__(self):
        self.popen_viewer.send_signal(signal.SIGINT)
        self.popen_vehicle.send_signal(signal.SIGINT)
        self.popen_vehicle1.send_signal(signal.SIGINT)
        self.popen_milee_autoware.send_signal(signal.SIGINT)
        self.popen_milee_autoware_interface.send_signal(signal.SIGINT)
        self.popen_traffic_signal.send_signal(signal.SIGINT)
        self.popen_user.send_signal(signal.SIGINT)
        # self.popen_shutter.send_signal(signal.SIGINT)
        # self.popen_light.send_signal(signal.SIGINT)

    def start(self):
        self.popen_viewer.wait()


if __name__ == '__main__':
    setproctitle("ams_launcher")
    launcher = Launcher()
    try:
        launcher.start()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(logger.pformat(e))
    finally:
        del launcher
