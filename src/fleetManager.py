#!/usr/bin/env python
# coding: utf-8

from time import time
import json
from eventLoop import EventLoop
from route import Route
from arrow import Arrow
from waypoint import Waypoint
from const import CONST

BUS_STOP_CIRCLE_RADIUS = 50.0  # [m]


class FleetManager(EventLoop):
    def __init__(self, waypoint, arrow, route):
        super().__init__()
        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route

        self.users = {}
        self.vehicles = {}  # vehicleID: {lat: [float], lng: [float], schedule: }
        self.trafficSignals = {}

        self.setSubscriber(CONST.TOPICS.USER_STATUS)
        self.setSubscriber(CONST.TOPICS.VEHICLE_STATUS)
        self.setSubscriber(CONST.TOPICS.TRAFFICSIGNAL_STATUS)
        self.setSubscriber(CONST.TOPICS.FLEET_STATUS)
        self.setOnMessageFunction(self.__onMessageFunction)

    def __onMessageFunction(self, client, userdata, topic, message):
        self.__client = client
        if topic == CONST.TOPICS.VEHICLE_STATUS:
            self.updateVehicleStatus(message)
        elif topic == CONST.TOPICS.USER_STATUS:
            self.updateUserStatus(message)
        elif topic == CONST.TOPICS.TRAFFICSIGNAL_STATUS:
            self.updateTrafficSignalStatus(message)
        elif topic == CONST.TOPICS.FLEET_STATUS:
            self.publishFleetStatus(message)
        else:
            print(topic, message)

    def updateUserStatus(self, message):
        userID, startWaypointID, goalWaypointID, state, vehicleID = message.split(",")
        userStatus = {
            "userID": userID,
            "startWaypointID": startWaypointID,
            "goalWaypointID": goalWaypointID,
            "state": state,
            "vehicleID": None if vehicleID == "None" else vehicleID,
        }

        if userStatus["state"] == CONST.USER_STATE.WAITING:
            self.dispatch(userStatus)

        self.users[userID] = userStatus

    def updateTrafficSignalStatus(self, message):
        trafficSignal = json.loads(message)
        self.trafficSignals[trafficSignal["entryExitRouteID"]] = trafficSignal

    def setVehicle(self, vehicle, vehicleSchedule):
        print("setVehicle {}".format(vehicle["vehicleID"]))
        self.vehicles[vehicle["vehicleID"]] = vehicle
        self.vehicles[vehicle["vehicleID"]]["schedules"] = [vehicleSchedule]

    def getDispatchableVehicles(self):
        return dict(filter(
            lambda x: x[1]["schedules"][0]["content"]["type"] in [CONST.VEHICLE_STATE.STANDBY],
            self.vehicles.items()
        ))

    def dispatch(self, userStatus):
        startPoint = {
            "arrowID": self.arrow.getArrowIDsFromWaypointID(userStatus["startWaypointID"])[0],
            "waypointID": userStatus["startWaypointID"],
        }
        vehicles = self.getDispatchableVehicles()
        goalPoints = []
        for vehicleID, goalWaypointID in map(lambda x: (x["vehicleID"], x["waypointID"]), vehicles.values()):
            goalPoints.append({
                "goalID": vehicleID,
                "arrowID": self.arrow.getArrowIDsFromWaypointID(goalWaypointID)[0],
                "waypointID": goalWaypointID,
            })
        routes = self.route.getShortestRoutes(startPoint, goalPoints, reverse=True)
        if len(routes) == 0:
            print("no pickUpRoute")
            return
        pickUpRoute = min(routes.items(), key=lambda x: x[1]["cost"])[1]



        startPoint = {
            "arrowID": self.arrow.getArrowIDsFromWaypointID(userStatus["startWaypointID"])[0],
            "waypointID": userStatus["startWaypointID"],
        }
        goalPoints = [{
            "goalID": userStatus["userID"],
            "arrowID": self.arrow.getArrowIDsFromWaypointID(userStatus["goalWaypointID"])[0],
            "waypointID": userStatus["goalWaypointID"],
        }]
        routes = self.route.getShortestRoutes(startPoint, goalPoints, reverse=False)
        if len(routes) == 0:
            print("cant carryRoute")
            return
        carryRoute = min(routes.items(), key=lambda x: x[1]["cost"])[1]



        print("dispatch", self.vehicles[pickUpRoute["goalID"]])
        if self.vehicles[pickUpRoute["goalID"]]["schedules"][-1]["content"]["type"] == "standBy":
            self.vehicles[pickUpRoute["goalID"]]["schedules"].pop()
        currentTime = time()
        self.vehicles[pickUpRoute["goalID"]]["schedules"].extend([
            {
                "scheduleID": "pickup",
                "startTime": currentTime,
                "endTime": currentTime + 1000,
                "content": {
                    "type": CONST.VEHICLE_STATE.MOVE,
                    "route": {
                        "goalWaypointID": pickUpRoute["startWaypointID"],
                        "arrowIDs": pickUpRoute["arrowIDs"]
                    }
                }
            },
            {
                "scheduleID": "takeOn",
                "startTime": currentTime + 1000,
                "endTime": currentTime + 1010,
                "content": {
                    "type": CONST.VEHICLE_STATE.STOP,
                    "stopTime": 10,
                }
            },
            {
                "scheduleID": "carry",
                "startTime": currentTime + 1010,
                "endTime": currentTime + 2010,
                "content": {
                    "type": CONST.VEHICLE_STATE.MOVE,
                    "route": {
                        "goalWaypointID": carryRoute["goalWaypointID"],
                        "arrowIDs": carryRoute["arrowIDs"]
                    }
                }
            },
            {
                "scheduleID": "discharge",
                "startTime": currentTime + 2010,
                "endTime": currentTime + 2020,
                "content": {
                    "type": CONST.VEHICLE_STATE.STOP,
                    "stopTime": 10,
                }
            },
            {
                "scheduleID": "standBy",
                "startTime": currentTime + 2020,
                "endTime": currentTime + 2020 + 86400,
                "content": {
                    "type": CONST.VEHICLE_STATE.STANDBY,
                }
            },
        ])

        self.publish(CONST.TOPICS.DIRECTIONS, json.dumps({
            "vehicleID": pickUpRoute["goalID"],
            "schedules": self.vehicles[pickUpRoute["goalID"]]["schedules"]
        }))

    def updateVehicleStatus(self, message):
        vehicle = json.loads(message)
        vehicleID = vehicle["vehicleID"]
        vehicleSchedule = vehicle.pop("schedule")
        if vehicleID not in self.vehicles:
            self.setVehicle(vehicle, vehicleSchedule)
        else:
            self.vehicles[vehicleID].update(vehicle)

            self.updateVehicleSchedule(vehicleID, vehicleSchedule)

        # user notice
        if self.vehicles[vehicleID]["schedules"][0]["content"]["type"] in [CONST.VEHICLE_STATE.STOP, CONST.VEHICLE_STATE.STANDBY]:
            message = ",".join([vehicleID, self.vehicles[vehicleID]["waypointID"]])
            self.publish(CONST.TOPICS.NOTICE, message)

    def updateVehicleSchedule(self, vehicleID, schedule):
        i = list(map(lambda x: x["scheduleID"], self.vehicles[vehicleID]["schedules"])).index(schedule["scheduleID"])
        self.vehicles[vehicleID]["schedules"] = self.vehicles[vehicleID]["schedules"][i:]

    def publishFleetStatus(self, message):
        if message == "":
            self.publish(CONST.TOPICS.FLEET_STATUS, json.dumps({
                "users": self.users,
                "vehicles": self.vehicles,
                "trafficSignals": self.trafficSignals
            }))


if __name__ == '__main__':
    waypoint = Waypoint()
    waypoint.load("../res/waypoint.json")

    arrow = Arrow()
    arrow.load("../res/arrow.json")

    route = Route()
    route.setWaypoint(waypoint)
    route.setArrow(arrow)
    try:
        route.load("../res/route.json")
    except:
        print("../res/route.json isnt exist")

    fleetManager = FleetManager(waypoint, arrow, route)
    fleetManager.start()
