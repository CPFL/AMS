#!/usr/bin/env python
# coding: utf-8

import random
from argparse import ArgumentParser
from time import time
from uuid import uuid1 as uuid

from ams import Waypoint, Arrow, Route, Schedule, Target, Spot
from ams.nodes import User, SimBusUser, SimBus

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-ID", "--id", type=str, default=None, help="node id")
parser.add_argument("-N", "--name", type=str, default="bu1", help="name")
parser.add_argument("-W", "--path_waypoint_json", type=str,
                    default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument("-A", "--path_arrow_json", type=str,
                    default="../../res/arrow.json", help="arrow.json path")
parser.add_argument("-PS", "--path_spot_json", type=str,
                    default="../../res/spot.json", help="spot.json path")
args = parser.parse_args()


if __name__ == '__main__':

    waypoint = Waypoint()
    waypoint.load(args.path_waypoint_json)

    arrow = Arrow(waypoint)
    arrow.load(args.path_arrow_json)

    spot = Spot()
    spot.load(args.path_spot_json)
    bus_parkable_spots = spot.get_spots_of_target_group(Target.new_node_target(SimBus))
    bus_stop_ids = list(bus_parkable_spots.keys())

    start_bus_stop_id = random.choice(bus_stop_ids)
    start_waypoint_id = bus_parkable_spots[start_bus_stop_id].contact.waypoint_id
    start_arrow_code = bus_parkable_spots[start_bus_stop_id].contact.arrow_code
    start_time = time() - 5

    bus_stop_ids.remove(start_bus_stop_id)

    goal_bus_stop_id = random.choice(bus_stop_ids)
    goal_waypoint_id = bus_parkable_spots[goal_bus_stop_id].contact.waypoint_id
    goal_arrow_code = bus_parkable_spots[goal_bus_stop_id].contact.arrow_code

    bus_user = SimBusUser(
        _id=args.id if args.id is not None else str(uuid()),
        name=args.name,
        dt=3.0
    )
    target_bus_user = Target.new_node_target(bus_user)
    trip_schedule = Schedule.new_schedule(
        [
            target_bus_user,
            Target.new_target(start_bus_stop_id, SimBusUser.CONST.TARGET_GROUP.START_BUS_STOP),
            Target.new_target(goal_bus_stop_id, SimBusUser.CONST.TARGET_GROUP.GOAL_BUS_STOP)
        ],
        "trip_schedule", start_time, start_time+9999,
        Route.new_route(start_waypoint_id, goal_waypoint_id, [start_arrow_code, goal_arrow_code])
    )
    bus_user.set_trip_schedules([trip_schedule])
    bus_user.set_schedules(Schedule.new_schedules([
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=User.CONST.TRIGGER.LOG_IN,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=SimBusUser.CONST.TRIGGER.WAIT,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=SimBusUser.CONST.TRIGGER.GET_ON,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=SimBusUser.CONST.TRIGGER.GOT_ON,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=SimBusUser.CONST.TRIGGER.MOVE_VEHICLE,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=SimBusUser.CONST.TRIGGER.REQUEST_STOP,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=SimBusUser.CONST.TRIGGER.GET_OUT,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=SimBusUser.CONST.TRIGGER.GOT_OUT,
            start_time=start_time,
            end_time=start_time + 1
        ),
        Schedule.new_schedule(
            targets=[target_bus_user],
            event=User.CONST.TRIGGER.LOG_OUT,
            start_time=start_time,
            end_time=start_time + 1
        ),
    ]))
    bus_user.start(host=args.host, port=args.port)
