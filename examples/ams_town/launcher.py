#!/usr/bin/env python
# coding: utf-8

import signal
from subprocess import Popen, check_call
from time import sleep

from setproctitle import setproctitle

from ams import logger


def killall_old_nodes():
    for proctitle in ["ams_launcher", "viewer", "ams_autoware", "ams_autoware_interface", "ams_vehicle"]:
        command = "killall -9 {}".format(proctitle)
        try:
            check_call(command.split(" "))
        except:
            pass


class Launcher(object):

    def __init__(self):
        self.processes = {}

        self.launch_postee()

        self.launch_milee()

        print("launch viewer")
        self.processes["viewer"] = Popen(["python", "viewer.py"])

    def __delete__(self):
        for process in self.processes.values():
            process.send_signal(signal.SIGINT)

    def launch_postee(self):
        print("launch vehicle1 postee1")
        command = "python ../node_launcher/vehicle.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/vehicle/postee1.json -SMP ./res/state_machines/vehicle.json"
        self.processes["vehicle1_postee1"] = Popen(command.split(" "))

        print("launch vehicle2 postee1")
        command = "python ../node_launcher/vehicle.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/vehicle/postee1.json -SMP ./res/state_machines/vehicle.json"
        self.processes["vehicle2_postee1"] = Popen(command.split(" "))

        print("launch autoware postee1")
        command = "python ../node_launcher/autoware.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/autoware/postee1.json -SMP ./res/state_machines/autoware.json -IF"
        self.processes["autoware_postee1"] = Popen(command.split(" "))

        print("launch autoware interface postee1")
        command = "python ../node_launcher/autoware_interface.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/autoware_interface/postee1.json -IF"
        self.processes["autoware_interface_postee1"] = Popen(command.split(" "))

        sleep(10)

        print("publish change_schedule event to postee1")
        command = "python publish_event.py -MT /sim/dispatcher/postee_route1/vehicle/postee1/event -EN change_schedule"
        check_call(command.split(" "))

        sleep(5)

        print("publish schedule to postee1")
        command = "python publish_schedule.py -DIFP ./res/initials/dispatcher/postee_route1.json -MH localhost -MP 1883 -MT /sim/dispatcher/postee_route1/vehicle/postee1/schedule -SID default"
        check_call(command.split(" "))

    def launch_milee(self):
        print("launch dispatcher milee_taxi")
        command = "python ../node_launcher/dispatcher.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/dispatcher/milee_taxi.json -SMP ./res/state_machines/dispatcher/milee_taxi.json"
        self.processes["dispatcher_milee_taxi"] = Popen(command.split(" "))

        # for vehicle_id in ["milee1", "milee2"]:
        for vehicle_id in ["milee1"]:
            print("launch vehicle {}".format(vehicle_id))
            command = "python ../node_launcher/vehicle.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/vehicle/{}.json -SMP ./res/state_machines/vehicle.json".format(vehicle_id)
            self.processes["vehicle_{}".format(vehicle_id)] = Popen(command.split(" "))

            print("launch autoware {}".format(vehicle_id))
            command = "python ../node_launcher/autoware.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/autoware/{}.json -SMP ./res/state_machines/autoware.json -IF".format(vehicle_id)
            self.processes["autoware_{}".format(vehicle_id)] = Popen(command.split(" "))

            print("launch autoware interface {}".format(vehicle_id))
            command = "python ../node_launcher/autoware_interface.py -KCT redis -PSCT paho -TD sim -IFP ./res/initials/autoware_interface/{}.json -IF".format(vehicle_id)
            self.processes["autoware_interface_{}".format(vehicle_id)] = Popen(command.split(" "))

        print("launch milee_taxi_user_generator")
        command = "python user_generator.py -UIDP ./res/initials/user -IF"
        self.processes["milee_taxi_user_generator"] = Popen(command.split(" "))

    def start(self):
        self.processes["viewer"].wait()


if __name__ == '__main__':
    killall_old_nodes()

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
