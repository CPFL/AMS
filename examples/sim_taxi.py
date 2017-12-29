#!/usr/bin/env python
# coding: utf-8

import sys
from time import time
from copy import deepcopy
from ams import Waypoint, Arrow, Route, Intersection, Topic
from ams.nodes import SimTaxi, Vehicle
from ams.messages import vehicle_message

WAYPOINT_FILE = "../res/waypoint.json"
ARROW_FILE = "../res/arrow.json"
INTERSECTION_FILE = "../res/intersection.json"


if __name__ == '__main__':
    waypoint = Waypoint()
    waypoint.load(WAYPOINT_FILE)

    arrow = Arrow(waypoint)
    arrow.load(ARROW_FILE)

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    intersection = Intersection()
    intersection.load(INTERSECTION_FILE)

    start_waypoint_id = "9566"  # 9232
    lat, lng = waypoint.get_latlng(start_waypoint_id)

    current_time = time()

    topic = Topic()
    topic.set_message(vehicle_message)
    schedules = deepcopy(topic.get_template()["schedules"])
    schedules[0]["start_time"] = current_time-5
    schedules[0]["duration"] = 10
    schedules[0]["action"] = Vehicle.ACTION.STOP
    # schedules[0]["route"] = None

    sim_taxi = SimTaxi(
        name=sys.argv[1],
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        intersection=intersection,
        waypoint_id=start_waypoint_id,
        velocity=0.00003333,
        schedules=schedules,
        dt=0.5
    )
    sim_taxi.start(host="localhost", port=1883)
