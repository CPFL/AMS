#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser
from ams import Waypoint, Arrow, Route, Spot, Schedule, Target, ScheduleBranch
from ams.nodes import SimBusFleet
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-N", "--name", type=str, default="tf1", help="name")
parser.add_argument("-PW", "--path_waypoint_json", type=str,
                    default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument("-PA", "--path_arrow_json", type=str,
                    default="../../res/arrow.json", help="arrow.json path")
parser.add_argument("-PS", "--path_spot_json", type=str,
                    default="../../res/spot.json", help="spot.json path")
parser.add_argument("-PBS", "--path_bus_schedule_json", type=str,
                    default="../../res/bus_schedule.json", help="bus_schedule.json path")
parser.add_argument("-WID", "--start_waypoint_id", type=str,
                    default=None, help="start waypoint id")
args = parser.parse_args()


def load_bus_schedule_json(path_bus_schedule_json):
    with open(path_bus_schedule_json, "r") as f:
        bus_schedules_json = json.load(f)["busSchedules"]
    bus_schedules = {}
    for target_bus_schedules in bus_schedules_json:
        schedule_branches = []
        for schedule_branch in target_bus_schedules["scheduleBranches"]:
            common_schedules = []
            for common in schedule_branch["common"]:
                common_schedules = Schedule.get_merged_schedules(common_schedules, [Schedule.new_schedule(
                    Target.new_targets(common["targets"]) if "targets" in common else None,
                    common["event"], None, None,
                    Route.decode_route_code(common["routeCode"])
                )])

            main_schedules = []
            for main in schedule_branch["main"]:
                main_schedules = Schedule.get_merged_schedules(
                    main_schedules, [Schedule.new_schedule(
                        Target.new_targets(main["targets"]) if "targets" in main else None,
                        main["event"], None, None,
                        Route.decode_route_code(main["routeCode"])
                    )]
                )
            sub_schedules = []
            for sub in schedule_branch["sub"]:
                sub_schedules = Schedule.get_merged_schedules(
                    sub_schedules, [Schedule.new_schedule(
                        Target.new_targets(sub["targets"]) if "targets" in sub else None,
                        sub["event"], None, None,
                        Route.decode_route_code(sub["routeCode"])
                    )]
                )
            schedule_branches.append(
                ScheduleBranch.new_schedule_branch(common_schedules, main_schedules, sub_schedules))
        bus_schedules[target_bus_schedules["ID"]] = schedule_branches
    return bus_schedules


if __name__ == '__main__':

    waypoint = Waypoint()
    waypoint.load(args.path_waypoint_json)

    arrow = Arrow(waypoint)
    arrow.load(args.path_arrow_json)

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    spot = Spot()
    spot.load(args.path_spot_json)

    bus_schedules = load_bus_schedule_json(args.path_bus_schedule_json)

    # pp(bus_schedules)

    bus_fleet = SimBusFleet(
        name=args.name,
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        spot=spot,
    )
    bus_fleet.set_bus_schedules(bus_schedules)
    bus_fleet.start(host=args.host, port=args.port)
