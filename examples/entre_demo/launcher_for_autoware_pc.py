#!/usr/bin/env python
# coding: utf-8

import signal
from subprocess import Popen, check_call
from time import sleep
from argparse import ArgumentParser

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

    def __init__(self, mqtt_broker_host="localhost", mqtt_broker_port=1883):
        self.processes = {}

        print("launch autoware interface logiee_s")
        command = "python ../node_launcher/autoware_interface.py -PSH {} -PSCT paho -TD sim -IFP ./res/initials/autoware_interface.json -URF".format(mqtt_broker_host)
        self.processes["autoware_interface_logiee_s"] = Popen(command.split(" "))

        print("launch vehicle logiee_s")
        command = "python ../node_launcher/vehicle.py -PSH {} -PSCT paho -TD sim -IFP ./res/initials/vehicle.json -SMP ./res/state_machines/vehicle.json".format(mqtt_broker_host)
        self.processes["vehicle_logiee_s"] = Popen(command.split(" "))

        sleep(5)

        print("publish change_schedule event to logiee_s")
        command = "python publish_event.py -MH {} -MP {} -MT /sim/dispatcher/entre_demo/vehicle/logiee_s/event -EN change_schedule".format(mqtt_broker_host, mqtt_broker_port)
        check_call(command.split(" "))

        sleep(2)

        print("publish schedule to logiee_s")
        command = "python publish_schedule.py -DIFP ./res/initials/dispatcher.json -MH {} -MP {} -MT /sim/dispatcher/entre_demo/vehicle/logiee_s/schedule -SID default".format(mqtt_broker_host, mqtt_broker_port)
        check_call(command.split(" "))

    def __del__(self):
        for process in self.processes.values():
            process.send_signal(signal.SIGINT)
            del process

    def start(self):
        self.processes["vehicle_logiee_s"].wait()


if __name__ == '__main__':
    killall_old_nodes()

    parser = ArgumentParser()
    parser.add_argument("-BH", "--broker_host", type=str, default="localhost", help="mqtt broker host")
    parser.add_argument("-BP", "--broker_port", type=int, default=1883, help="mqtt broker port")
    args = parser.parse_args()

    setproctitle("ams_launcher")
    launcher = Launcher(args.broker_host, args.broker_port)
    try:
        launcher.start()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(logger.pformat(e))
    finally:
        del launcher
