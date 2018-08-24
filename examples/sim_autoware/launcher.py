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

        print("launch milee")
        command = "python ../node_launchers/sim_autoware.py -PSCT paho -TD k_turn -NN ros -ID mkr1 -VNN milee_k_turn " + \
                  "-VID mk1 -IP 3731.9097 -99401.8359 85.7022 0.0 0.0 0.011330634207471232 0.999935806303813"
        self.popen_milee = Popen(command.split(" "))

    def __del__(self):
        self.popen_viewer.terminate()
        self.popen_milee.terminate()

    def start(self):
        self.popen_viewer.wait()


if __name__ == '__main__':
    lanucher = Launcher()
    try:
        lanucher.start()
    except KeyboardInterrupt:
        traceback.print_exc()
        del lanucher
