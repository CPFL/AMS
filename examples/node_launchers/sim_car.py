#!/usr/bin/env python
# coding: utf-8

import random
from time import time
from argparse import ArgumentParser

from ams import Waypoint, Arrow, Intersection, Route, Schedule
from ams.nodes import Vehicle, SimCar

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-N", "--name", type=str, default="st1", help="name")
parser.add_argument("-W", "--path_waypoint_json", type=str,
                    default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument("-A", "--path_arrow_json", type=str,
                    default="../../res/arrow.json", help="arrow.json path")
parser.add_argument("-I", "--path_intersection_json", type=str,
                    default="../../res/intersection.json", help="intersection.json path")
parser.add_argument("-SWID", "--start_waypoint_id", type=str,
                    default=None, help="start waypoint id")
parser.add_argument("-GWID", "--goal_waypoint_id", type=str,
                    default=None, help="goal waypoint id")
args = parser.parse_args()


if __name__ == '__main__':

    waypoint = Waypoint()
    waypoint.load(args.path_waypoint_json)

    arrow = Arrow(waypoint)
    arrow.load(args.path_arrow_json)

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    intersection = Intersection()
    intersection.load(args.path_intersection_json)

    start_waypoint_id = args.start_waypoint_id
    if start_waypoint_id is None:
        start_waypoint_id = random.choice([  # "9566"  # 9232
            "8910", "8911", "8912", "8913", "8914", "8915", "8916", "8917", "8918", "8919", "8920", "8921", "8922", "8923", "8924", "8925", "8926",
            "9362", "9363", "9364", "9365", "9366", "9367", "9368", "9369", "9370", "9371", "9372", "9373", "9374", "9375", "9376", "9377",
            "8883", "8884", "8885", "8886", "8887", "8888", "8889", "8890", "8891", "8892", "8893", "8894", "8895", "8896", "8897",
            "9392", "9393", "9394", "9395", "9396", "9397", "9398", "9399", "9400", "9401", "9402", "9403", "9404",
            "10350", "10351", "10352", "10353", "10354", "10355", "10356", "10357", "10358", "10359", "10360", "10361", "10362", "10363", "10364", "10365", "10366", "10367", "10368", "10369", "10370", "10371", "10372", "10373", "10374",
            "9697", "9698", "9699", "9700", "9701", "9702", "9703", "9704", "9705", "9706", "9707", "9708",
            "8936", "8937", "8938", "8939", "8940", "8941", "8942", "8943", "8944", "8945", "8946", "8947", "8948", "8949", "8950", "8951", "8952", "8953", "8954", "8955", "8956", "8957", "8958", "8959", "8960", "8961", "8962", "8963", "8964", "8965", "8966", "8967", "8968"])
    start_arrow_code = arrow.get_arrow_codes_from_waypoint_id(start_waypoint_id)[0]
    current_time = time()

    schedule = Schedule.get_schedule(
        Vehicle.ACTION.STOP,
        current_time - 5,
        current_time + 5,
        Route.get_route(
            start_waypoint_id,
            start_waypoint_id,
            [start_arrow_code]
        )
    )
    schedules = Schedule.get_merged_schedules([], [schedule])

    if args.goal_waypoint_id is not None:
        start_point = {
            "arrow_code": start_arrow_code,
            "waypoint_id": start_waypoint_id,
        }
        goal_waypoint_id = args.goal_waypoint_id
        goal_arrow_code = arrow.get_arrow_codes_from_waypoint_id(goal_waypoint_id)[0]

        goal_id = "route" + goal_waypoint_id
        goal_points = [{
            "goal_id": goal_id,
            "arrow_code": goal_arrow_code,
            "waypoint_id": goal_waypoint_id,
        }]

        routes = route.get_shortest_routes(start_point, goal_points, reverse=False)

        schedule = Schedule.get_schedule(
            Vehicle.ACTION.MOVE,
            schedules[-1].period.end,
            schedules[-1].period.end + 100,
            Route.get_route(
                routes[goal_id]["start_waypoint_id"],
                routes[goal_id]["goal_waypoint_id"],
                routes[goal_id]["arrow_codes"]
            )
        )
        schedules = Schedule.get_merged_schedules(schedules, [schedule])
        schedule = Schedule.get_schedule(
            Vehicle.ACTION.STOP,
            schedules[-1].period.end,
            schedules[-1].period.end + 86400,
            Route.get_route(
                goal_waypoint_id,
                goal_waypoint_id,
                [goal_arrow_code],
            )
        )
    else:
        next_start_waypoint_id = start_waypoint_id
        next_start_arrow_code = start_arrow_code
        for i in range(10):
            start_point = {
                "arrow_code": next_start_arrow_code,
                "waypoint_id": next_start_waypoint_id,
            }
            goal_waypoint_id = random.choice([
                "8910", "8911", "8912", "8913", "8914", "8915", "8916", "8917", "8918", "8919", "8920", "8921", "8922", "8923", "8924", "8925", "8926",
                "9362", "9363", "9364", "9365", "9366", "9367", "9368", "9369", "9370", "9371", "9372", "9373", "9374", "9375", "9376", "9377",
                "8883", "8884", "8885", "8886", "8887", "8888", "8889", "8890", "8891", "8892", "8893", "8894", "8895", "8896", "8897",
                "9392", "9393", "9394", "9395", "9396", "9397", "9398", "9399", "9400", "9401", "9402", "9403", "9404",
                "9875", "9876", "9877", "9878", "9879", "9880", "9881", "9882", "9883", "9884", "9885", "9886", "9887",
                "9922", "9923", "9924", "9925", "9926", "9927", "9928", "9929", "9930",
                "9930", "9931", "9932", "9933", "9934", "9935",
                "10350", "10351", "10352", "10353", "10354", "10355", "10356", "10357", "10358", "10359", "10360", "10361", "10362", "10363", "10364", "10365", "10366", "10367", "10368", "10369", "10370", "10371", "10372", "10373", "10374",
                "9697", "9698", "9699", "9700", "9701", "9702", "9703", "9704", "9705", "9706", "9707", "9708",
                "8936", "8937", "8938", "8939", "8940", "8941", "8942", "8943", "8944", "8945", "8946", "8947", "8948", "8949", "8950", "8951", "8952", "8953", "8954", "8955", "8956", "8957", "8958", "8959", "8960", "8961", "8962", "8963", "8964", "8965", "8966", "8967", "8968",
                "9144", "9145", "9146", "9147", "9148", "9149", "9150", "9151",
            ])
            if goal_waypoint_id == next_start_waypoint_id:
                continue
            goal_arrow_code = arrow.get_arrow_codes_from_waypoint_id(goal_waypoint_id)[0]

            goal_id = "route" + goal_waypoint_id
            goal_points = [{
                "goal_id": goal_id,
                "arrow_code": goal_arrow_code,
                "waypoint_id": goal_waypoint_id,
            }]

            routes = route.get_shortest_routes(start_point, goal_points, reverse=False)

            schedule = Schedule.get_schedule(
                Vehicle.ACTION.MOVE,
                schedules[-1].period.end,
                schedules[-1].period.end + 100,
                Route.get_route(
                    routes[goal_id]["start_waypoint_id"],
                    routes[goal_id]["goal_waypoint_id"],
                    routes[goal_id]["arrow_codes"]
                )
            )
            schedules = Schedule.get_merged_schedules(schedules, [schedule])

            next_start_waypoint_id = goal_waypoint_id
            next_start_arrow_code = goal_arrow_code

        schedule = Schedule.get_schedule(
            Vehicle.ACTION.STOP,
            schedules[-1].period.end,
            schedules[-1].period.end + 86400,
            Route.get_route(
                next_start_waypoint_id,
                next_start_waypoint_id,
                [next_start_arrow_code],
            )
        )
    schedules = Schedule.get_merged_schedules(schedules, [schedule])

    sim_car = SimCar(
        name=args.name,
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        intersection=intersection,
        waypoint_id=start_waypoint_id,
        arrow_code=start_arrow_code,
        velocity=3.0,
        schedules=schedules,
        dt=0.5
    )
    sim_car.start(host=args.host, port=args.port)
