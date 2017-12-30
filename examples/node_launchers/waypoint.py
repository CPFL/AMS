#!/usr/bin/env python
# coding: utf-8


if __name__ == '__main__':
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2).pprint

    waypoint = Waypoint()
    waypoint.load("./res/waypoint.json")

    waypointIDs = waypoint.get_waypoint_ids()
    waypoints = waypoint.get_waypoints()
    pp(list(map(lambda x: (waypoints[x]["lat"], waypoints[x]["lng"]), waypointIDs)))

    # with open("./waypointsLatLng.json", "w") as f:
    #     json.dump(list(map(lambda x: (waypoints[x]["lat"], waypoints[x]["lng"]), waypointIDs)), f, indent="  ")
