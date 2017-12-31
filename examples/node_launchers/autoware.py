#!/usr/bin/env python
# coding: utf-8

from time import time
from ams import Waypoint, Arrow, Route, Intersection
from ams.nodes import Autoware

WAYPOINT_FILE = "../../res/waypoint.json"
ARROW_FILE = "../../res/arrow.json"
INTERSECTION_FILE = "../../res/intersection.json"


if __name__ == "__main__":

    start_waypoint_id = "9566"  # "8809"  # "9566"  # 9232

    waypoint = Waypoint()
    waypoint.load(WAYPOINT_FILE)

    arrow = Arrow(waypoint)
    arrow.load(ARROW_FILE)

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    intersection = Intersection()
    intersection.load(INTERSECTION_FILE)

    current_time = time()

    schedules = [
        {
            "scheduleID": "start",
            "startTime": current_time - 5,
            "endTime": current_time + 5,
            "content": {
                "type": "standBy",
            }
        },
    ]

    autoware = Autoware(
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        intersection=intersection,
        waypoint_id=start_waypoint_id,
        velocity=0.00001,
        schedules=schedules,
        dt=0.5
    )
    autoware.start(host="localhost", port=1883)
