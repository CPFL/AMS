#!/usr/bin/env python
# coding: utf-8

import signal
from subprocess import Popen, check_call
from time import sleep

from setproctitle import setproctitle

from ams import logger


def killall_old_nodes():
    for proctitle in ["ams_launcher", "ams_viewer", "ams_autoware", "ams_autoware_interface", "ams_vehicle"]:
        command = "killall -9 {}".format(proctitle)
        try:
            check_call(command.split(" "))
        except:
            pass


class Launcher(object):

    def __init__(self):
        self.processes = {}

        print("launch viewer")
        self.processes["viewer"] = Popen(["python", "viewer.py"])

        print("launch autoware logiee_s")
        command = "python ../node_launcher/autoware.py -PSCT paho -TD sim -IFP ./res/initials/autoware.json -SMP ./res/state_machines/autoware.json -IF"
        self.processes["autoware_logiee_s"] = Popen(command.split(" "))

        print("launch autoware interface logiee_s")
        command = "python ../node_launcher/autoware_interface.py -PSCT paho -TD sim -IFP ./res/initials/autoware_interface.json -IF"
        self.processes["autoware_interface_logiee_s"] = Popen(command.split(" "))

        print("launch vehicle logiee_s")
        command = "python ../node_launcher/vehicle.py -PSCT paho -TD sim -IFP ./res/initials/vehicle.json -SMP ./res/state_machines/vehicle.json"
        self.processes["vehicle_logiee_s"] = Popen(command.split(" "))

        sleep(5)

        print("publish change_schedule event to logiee_s")
        command = "python publish_event.py -MT /sim/dispatcher/entre_demo/vehicle/logiee_s/event -EN change_schedule"
        check_call(command.split(" "))

        sleep(2)

        print("publish schedule to logiee_s")
        command = "python publish_schedule.py -DIFP ./res/initials/dispatcher.json -MH localhost -MP 1883 -MT /sim/dispatcher/entre_demo/vehicle/logiee_s/schedule -SID default"
        check_call(command.split(" "))

    def __del__(self):
        for process in self.processes.values():
            process.send_signal(signal.SIGINT)
            del process

    def start(self):
        self.processes["vehicle_logiee_s"].wait()


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
