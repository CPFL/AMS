#!/usr/bin/env python
# coding: utf-8

import sys
import random
from time import time
from copy import deepcopy

from ams import Waypoint, Arrow, Intersection, Route, Topic
from ams.nodes import Vehicle, SimCar
from ams.messages import vehicle_message

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint

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

    currentTime = time()

    topic = Topic()
    topic.set_message(vehicle_message)
    schedules = deepcopy(topic.get_template()["schedules"])
    schedules[0]["start_time"] = currentTime-5
    schedules[0]["duration"] = 10
    schedules[0]["action"] = Vehicle.ACTION.STOP
    # schedules[0]["route"] = None


    # """
    next_start_waypoint_id = start_waypoint_id
    for i in range(10):
        startPoint = {
            "arrow_id": arrow.get_arrow_ids_from_waypoint_id(next_start_waypoint_id)[0],
            "waypoint_id": next_start_waypoint_id,
        }
        goalWaypointID = random.choice([
            "8910", "8911", "8912", "8913", "8914", "8915", "8916", "8917", "8918", "8919", "8920", "8921", "8922", "8923", "8924", "8925", "8926",
            "9362", "9363", "9364", "9365", "9366", "9367", "9368", "9369", "9370", "9371", "9372", "9373", "9374", "9375", "9376", "9377",
            "8883", "8884", "8885", "8886", "8887", "8888", "8889", "8890", "8891", "8892", "8893", "8894", "8895", "8896", "8897",
            "9392", "9393", "9394", "9395", "9396", "9397", "9398", "9399", "9400", "9401", "9402", "9403", "9404",
            "9875", "9876", "9877", "9878", "9879", "9880", "9881", "9882", "9883", "9884", "9885", "9886", "9887",
            # "9908", "9909", "9910", "9911", "9912", "9913", "9914", "9915", "9916", "9917", "9918", "9919", "9920", "9921",
            "9922", "9923", "9924", "9925", "9926", "9927", "9928", "9929", "9930",
            "9930", "9931", "9932", "9933", "9934", "9935",
            "10350", "10351", "10352", "10353", "10354", "10355", "10356", "10357", "10358", "10359", "10360", "10361", "10362", "10363", "10364", "10365", "10366", "10367", "10368", "10369", "10370", "10371", "10372", "10373", "10374",
            "9697", "9698", "9699", "9700", "9701", "9702", "9703", "9704", "9705", "9706", "9707", "9708",
            "8936", "8937", "8938", "8939", "8940", "8941", "8942", "8943", "8944", "8945", "8946", "8947", "8948", "8949", "8950", "8951", "8952", "8953", "8954", "8955", "8956", "8957", "8958", "8959", "8960", "8961", "8962", "8963", "8964", "8965", "8966", "8967", "8968",
            "9144", "9145", "9146", "9147", "9148", "9149", "9150", "9151",
        ])
        if goalWaypointID == next_start_waypoint_id:
            continue

        goalID = "route" + goalWaypointID
        goalPoints = [{
            "goal_id": goalID,
            "arrow_id": arrow.get_arrow_ids_from_waypoint_id(goalWaypointID)[0],
            "waypoint_id": goalWaypointID,
        }]
        # pp([startPoint, goalPoints])

        routes = route.get_shortest_routes(startPoint, goalPoints, reverse=False)
        # pp(routes)

        schedules.extend(deepcopy(topic.get_template()["schedules"]))
        schedules[-1]["start_time"] = 1
        schedules[-1]["duration"] = 1
        schedules[-1]["action"] = Vehicle.ACTION.MOVE
        schedules[-1]["route"]["start"]["waypoint_id"] = next_start_waypoint_id
        schedules[-1]["route"]["goal"]["waypoint_id"] = routes[goalID]["goal_waypoint_id"]
        schedules[-1]["route"]["arrow_ids"] = routes[goalID]["arrow_ids"]
        print(schedules[-1]["action"], schedules[-1]["route"]["arrow_ids"][0], schedules[-1]["route"]["arrow_ids"][-1])

        next_start_waypoint_id = routes[goalID]["goal_waypoint_id"]

    schedules.extend(deepcopy(topic.get_template()["schedules"]))
    schedules[-1]["start_time"] = 1
    schedules[-1]["duration"] = 1
    schedules[-1]["action"] = Vehicle.ACTION.STOP
    # """

    pp(schedules)

    sim_car = SimCar(
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
    sim_car.start(host="localhost", port=1883)
