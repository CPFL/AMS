#!/usr/bin/env python
# coding: utf-8

import random
import multiprocessing.managers
from multiprocessing import Process
from ams import get_ams_dict_client_class
from time import sleep


NUM_PROCESSES = 10


def src_setter(d):
    while True:
        sleep(2.0*random.random())
        d.set("src", {"test": "src"})


def dst_setter(d):
    while True:
        sleep(2.0*random.random())
        src = d.get("src")
        if src is not None:
            src["test"] = "dst"
            d.set("dst", src, "src")


def monitor(d):
    while True:
        sleep(1)
        print("monitor", d.keys())


if __name__ == "__main__":
    DC = get_ams_dict_client_class(multiprocessing)
    d = DC()

    p_monitor = Process(target=monitor, args=(d,))
    p_monitor.start()

    p_src_setters = {}
    for i in range(0, NUM_PROCESSES):
        p_src_setters[i] = Process(target=src_setter, args=(d,))
        p_src_setters[i].start()

    p_dst_setters = {}
    for i in range(0, NUM_PROCESSES):
        p_dst_setters[i] = Process(target=dst_setter, args=(d,))
        p_dst_setters[i].start()

    p_monitor.join()
