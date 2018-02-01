#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from subprocess import Popen
import json

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-I", "--path_intersection_json", type=str,
                    default="../../res/intersection.json", help="intersection.json path")
parser.add_argument("-C", "--path_cycle_json", type=str,
                    default=None, help="cycle.json path")
args = parser.parse_args()


if __name__ == '__main__':

    with open(args.path_intersection_json, "r") as f:
        intersections = json.load(f)["intersections"]

    cycles = {}
    if args.path_cycle_json is not None:
        with open(args.path_cycle_json, "r") as f:
            cycles = json.load(f)["cycles"]

    traffic_signals = {}
    for intersection_id, intersection in intersections.items():
        for route_code, traffic_signal in intersection["trafficSignals"].items():
            traffic_signals[route_code] = Popen(map(str, [
                "python", "../node_launchers/traffic_signal.py",
                "--host", args.host,
                "--port", args.port,
                "--route_code", route_code,
                "-C", json.dumps(cycles[traffic_signal["cycleID"]])
                # "--schedules", json.dumps(traffic_signal["schedules"])
            ]))

    for traffic_signal in traffic_signals.values():
        traffic_signal.wait()
