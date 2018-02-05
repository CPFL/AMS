#!/usr/bin/env python
# coding: utf-8

import sys
from time import time
from copy import deepcopy
import random
from ams import Waypoint, Arrow, Route, Intersection, Topic
from ams.nodes import SimTaxi, Vehicle
from ams.messages import vehicle_message

WAYPOINT_FILE = "../../res/waypoint.json"
ARROW_FILE = "../../res/arrow.json"
INTERSECTION_FILE = "../../res/intersection.json"


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

    start_waypoint_id = random.choice([  # "9566"  # 9232
        "8910", "8911", "8912", "8913", "8914", "8915", "8916", "8917", "8918", "8919", "8920", "8921", "8922", "8923", "8924", "8925", "8926",
        "9362", "9363", "9364", "9365", "9366", "9367", "9368", "9369", "9370", "9371", "9372", "9373", "9374", "9375", "9376", "9377",
        "8883", "8884", "8885", "8886", "8887", "8888", "8889", "8890", "8891", "8892", "8893", "8894", "8895", "8896", "8897",
        "9392", "9393", "9394", "9395", "9396", "9397", "9398", "9399", "9400", "9401", "9402", "9403", "9404",
        "10350", "10351", "10352", "10353", "10354", "10355", "10356", "10357", "10358", "10359", "10360", "10361", "10362", "10363", "10364", "10365", "10366", "10367", "10368", "10369", "10370", "10371", "10372", "10373", "10374",
        "9697", "9698", "9699", "9700", "9701", "9702", "9703", "9704", "9705", "9706", "9707", "9708",
        "8936", "8937", "8938", "8939", "8940", "8941", "8942", "8943", "8944", "8945", "8946", "8947", "8948", "8949", "8950", "8951", "8952", "8953", "8954", "8955", "8956", "8957", "8958", "8959", "8960", "8961", "8962", "8963", "8964", "8965", "8966", "8967", "8968"])

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
        velocity=3.0,
        schedules=schedules,
        dt=0.5
    )
    sim_taxi.start(host="localhost", port=1883)
