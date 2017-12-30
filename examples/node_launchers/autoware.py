#!/usr/bin/env python
# coding: utf-8

from ams import Waypoint, Arrow, Route, Intersection
from ams.nodes import Autoware

if __name__ == "__main__":

    startWaypointID = "9566"  # "8809"  # "9566"  # 9232
    waypoint = Waypoint()
    waypoint.load("../res/waypoint.json")
    lat, lng = waypoint.get_latlng(startWaypointID)

    from time import time
    currentTime = time()

    schedules = [
        {
            "scheduleID": "start",
            "startTime": currentTime - 5,
            "endTime": currentTime + 5,
            "content": {
                "type": "standBy",
            }
        },
    ]

    waypoint = Waypoint()
    waypoint.load("../res/waypoint.json")

    arrow = Arrow(waypoint)
    arrow.load("../res/arrow.json")

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    intersection = Intersection()
    intersection.load("../res/intersection.json")

    autoware = Autoware(
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        intersection=intersection,
        waypoint_id=startWaypointID,
        velocity=0.00001,
        schedules=schedules,
        dt=0.5
    )
    autoware.start()
