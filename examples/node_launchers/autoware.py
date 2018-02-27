#!/usr/bin/env python
# coding: utf-8

from time import time
from argparse import ArgumentParser
from ams import Waypoint, Arrow, Route, Schedule, Target
from ams.nodes import Autoware


parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-N", "--name", type=str, default="sim_car 1", help="name")
parser.add_argument("-W", "--path_waypoint_json", type=str,
                    default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument("-A", "--path_arrow_json", type=str,
                    default="../../res/arrow.json", help="arrow.json path")
args = parser.parse_args()


if __name__ == "__main__":

    waypoint = Waypoint()
    waypoint.load(args.path_waypoint_json)

    arrow = Arrow(waypoint)
    arrow.load(args.path_arrow_json)

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    current_time = time()

    autoware = Autoware(
        name=args.name,
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        dt=0.5
    )
    autoware.set_schedules([Schedule.new_schedule(
        [Target.new_node_target(autoware)],
        Autoware.CONST.STATE.STOP, current_time, current_time+100,
        None
    )])
    autoware.start(host=args.host, port=args.port)
