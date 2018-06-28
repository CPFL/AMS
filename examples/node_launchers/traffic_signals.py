#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from subprocess import Popen
import json

from ams.maps import Intersection


parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-C", "--path_cycle_json", type=str,
                    default="../../res/cycle.json", help="cycle.json path")
parser.add_argument("-I", "--path_intersection_json", type=str,
                    default="../../res/intersection.json", help="intersection.json path")
args = parser.parse_args()


if __name__ == '__main__':

    intersection = Intersection()
    intersection.load(args.path_intersection_json)

    with open(args.path_cycle_json, "r") as f:
        data = json.load(f)
        cycles = data["cycles"]

    traffic_signals = []

    try:
        for intersection_id in intersection.get_intersection_ids():
            for route_code, traffic_signal_configs in intersection.get_traffic_signals(intersection_id).items():
                cycle_option = []
                if traffic_signal_configs["cycleID"] is not None:
                    cycle_option = ["--cycle", json.dumps(cycles[traffic_signal_configs["cycleID"]])]

                traffic_signals.append({
                   "route_code": route_code,
                   "process": Popen([
                        "python", "../node_launchers/traffic_signal.py",
                        "--host", args.host,
                        "--port", str(args.port),
                        "--id", route_code,
                        "--route_code", route_code,
                        # "--schedules", json.dumps(traffic_signal_configs["schedules"])
                    ] + cycle_option)
                })

        traffic_signals[0]["process"].wait()
    except:
        for traffic_signal in traffic_signals:
            traffic_signal["process"].terminate()
