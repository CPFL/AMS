#!/usr/bin/env python
# coding: utf-8

from time import time
from argparse import ArgumentParser

from ams import Waypoint, Arrow, Route, Schedule, Target
from ams.nodes import AutowareVoiceRecognition, Vehicle


parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
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

    autoware_voice_recognition = AutowareVoiceRecognition(
        _id=args.id,
        name=args.name,
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        dt=0.5
    )

    start_waypoint_id = "8916"
    goal_waypoint_id = "9703"

    current_time = time()

    vehicle_schedules = list()
    vehicle_schedule = Schedule.new_schedule(
        [Target.new_node_target(autoware_voice_recognition)],
        AutowareVoiceRecognition.CONST.STATE.STOP, current_time, current_time + 100,
        None)
    vehicle_schedules = Schedule.get_merged_schedules(
        vehicle_schedules, [vehicle_schedule])

    start_point = {
        "arrow_code": arrow.get_arrow_codes_from_waypoint_id(start_waypoint_id)[0],
        "waypoint_id": start_waypoint_id,
    }
    goal_id = "route" + goal_waypoint_id
    goal_points = [{
        "goal_id": goal_id,
        "arrow_code": arrow.get_arrow_codes_from_waypoint_id(goal_waypoint_id)[0],
        "waypoint_id": goal_waypoint_id,
    }]
    shortest_routes = route.get_shortest_routes(start_point, goal_points, reverse=False)
    shortest_route = shortest_routes[goal_id]
    shortest_route.pop("cost")
    shortest_route.pop("goal_id")

    vehicle_schedule = Schedule.new_schedule(
        [Target.new_node_target(autoware_voice_recognition)],
        Vehicle.CONST.ACTION.MOVE, current_time, current_time + 100,
        shortest_route
    )

    vehicle_schedules = Schedule.get_merged_schedules(
        vehicle_schedules, [vehicle_schedule])

    autoware_voice_recognition.set_schedules(vehicle_schedules)

    start_arrow_code = arrow.get_arrow_codes_from_waypoint_id(start_waypoint_id)[0]
    autoware_voice_recognition.set_waypoint_id_and_arrow_code(start_waypoint_id, start_arrow_code)
    autoware_voice_recognition.set_velocity(3.0)
    autoware_voice_recognition.start(host=args.host, port=args.port)
