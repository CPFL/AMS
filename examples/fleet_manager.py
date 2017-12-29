#!/usr/bin/env python
# coding: utf-8

from ams import Waypoint, Arrow, Route
from ams.nodes import FleetManager

WAYPOINT_FILE = "../res/waypoint.json"
ARROW_FILE = "../res/arrow.json"
ROUTE_FILE = "../res/route.json"

if __name__ == '__main__':
    waypoint = Waypoint()
    waypoint.load(WAYPOINT_FILE)

    arrow = Arrow(waypoint)
    arrow.load(ARROW_FILE)

    route = Route()
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)
    try:
        route.load(ROUTE_FILE)
    except:
        print("no", ROUTE_FILE)

    fleetManager = FleetManager(waypoint, arrow, route)
    fleetManager.start(host="localhost", port=1883)
