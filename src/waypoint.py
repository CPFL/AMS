#!/usr/bin/env python
# coding: utf-8
import json


class Waypoint(object):
    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.setWaypoints(data["waypoints"])
        return True

    def connectToRedis(self, host, port, dbname):
        return None

    def setWaypoints(self, waypoints):
        self.__waypoints = waypoints

    def getWaypoints(self):
        return self.__waypoints

    def getWaypointIDs(self):
        return list(self.__waypoints.keys())

    def getLatLng(self, waypointID):
        return self.__waypoints[waypointID]["lat"], self.__waypoints[waypointID]["lng"]

    def getWaypoint(self, waypointID):
        return self.__waypoints[waypointID]


if __name__ == '__main__':
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2).pprint

    waypoint = Waypoint()
    waypoint.load("./res/waypoint.json")

    waypointIDs = waypoint.getWaypointIDs()
    waypoints = waypoint.getWaypoints()
    pp(list(map(lambda x: (waypoints[x]["lat"], waypoints[x]["lng"]), waypointIDs)))

    # with open("./waypointsLatLng.json", "w") as f:
    #     json.dump(list(map(lambda x: (waypoints[x]["lat"], waypoints[x]["lng"]), waypointIDs)), f, indent="  ")
