#!/usr/bin/env python
# coding: utf-8

import sys
from copy import deepcopy
from time import time
from ams import Waypoint, Arrow, Route, Topic
from ams.nodes import Vehicle, AutowareTaxi
from ams.messages import vehicle_message

WAYPOINT_FILE = "../../res/waypoint.json"
ARROW_FILE = "../../res/arrow.json"


if __name__ == "__main__":
    start_waypoint_id = "9566"  # "8809"  # "9566"  # 9232

    waypoint = Waypoint()
    waypoint.load(WAYPOINT_FILE)
    arrow = Arrow(waypoint)
    arrow.load(ARROW_FILE)
    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)

    current_time = time()

    topic = Topic()
    topic.set_message(vehicle_message)
    schedules = deepcopy(topic.get_template()["schedules"])
    schedules[0]["start_time"] = current_time-5
    schedules[0]["duration"] = 10
    schedules[0]["action"] = Vehicle.ACTION.STOP

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
    autoware_taxi.start(host="localhost", port=1883)
