#!/usr/bin/env python
# coding: utf-8

import sys
from waypoint import Waypoint
from route import Route
if __name__ == "__main__":
    waypoint = Waypoint()
    waypoint.load("../res/waypoint.json")
    arrow = Arrow(waypoint)
    arrow.load("../res/arrow.json")
    route = Route()
    route.setWaypoint(waypoint)
    route.setArrow(arrow)

    start_waypoint_id = "9566"  # "8809"  # "9566"  # 9232

    lat, lng = waypoint.getLatLng(start_waypoint_id)

    from time import time
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

    autoware_taxi = AutowareTaxi(
        name=sys.argv[1],
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        waypoint_id=start_waypoint_id,
        velocity=0.00001,
        schedules=schedules,
        dt=0.5
    )
    autoware_taxi.start()
