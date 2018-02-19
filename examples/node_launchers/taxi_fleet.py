#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from ams import Waypoint, Arrow, Route
from ams.nodes import TaxiFleet

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-N", "--name", type=str, default="tf1", help="name")
parser.add_argument("-W", "--path_waypoint_json", type=str,
                    default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument("-A", "--path_arrow_json", type=str,
                    default="../../res/arrow.json", help="arrow.json path")
parser.add_argument("-I", "--path_intersection_json", type=str,
                    default="../../res/intersection.json", help="intersection.json path")
parser.add_argument("-WID", "--start_waypoint_id", type=str,
                    default=None, help="start waypoint id")
args = parser.parse_args()


if __name__ == '__main__':

    waypoint = Waypoint()
    waypoint.load(args.path_waypoint_json)

    arrow = Arrow(waypoint)
    arrow.load(args.path_arrow_json)

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    taxi_fleet = TaxiFleet(
        name=args.name,
        waypoint=waypoint,
        arrow=arrow,
        route=route
    )
    taxi_fleet.start(host=args.host, port=args.port)
