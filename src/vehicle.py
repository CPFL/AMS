#!/usr/bin/env python
# coding: utf-8

from time import time, sleep
import json
from eventLoop import EventLoop
from waypoint import Waypoint
from arrow import Arrow
from route import Route
from intersection import Intersection
from const import CONST

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint

RIDE_CAPACITY = 20  # passengers
LOWER_INTER_VEHICLE_DISTANCE = 5.0
LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE = 2.0

class Vehicle(EventLoop):
    def __init__(self, vehicleID, waypoint, arrow, route, intersection, waypointID, velocity, numOfPassengers=0, schedules=None, dt=1.0):
        super().__init__()
        self.__meterPerLat = (1.11 / 0.00001)
        self.__meterPerLng = (9.05 / 0.0001)
        self.stayTime = 10.0

        self.vehicleID = vehicleID
        self.waypointID = waypointID
        self.velocity = velocity

        self.numOfPassengers = None
        self.schedules = schedules

        self.trafficSignals = {}

        self.otherVehicles = {}

        self.dt = dt

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.intersection = intersection

        self.arrowID = self.arrow.getArrowIDsFromWaypointID(waypointID)[0]
        self.lat, self.lng = self.waypoint.getLatLng(self.waypointID)
        self.heading = self.arrow.getHeading(self.arrowID, self.waypointID)


        self.state = CONST.VEHICLE_STATE.STOP
        self.__stayEndTime = time() + self.stayTime

        self.setOnMessageFunction(
            onMessageFunction=self.__onMessageFunction,
            userData={
                "setDirections": self.setDirections,
                "setOtherVehiclePoses": self.setOtherVehiclePoses,
                "setTrafficSignals": self.setTrafficSignals,
            }
        )
        self.setSubscriber(CONST.TOPICS.DIRECTIONS)
        self.setSubscriber(CONST.TOPICS.VEHICLE_STATUS)
        self.setSubscriber(CONST.TOPICS.TRAFFICSIGNAL_STATUS)
        self.setMainLoop(self.__mainLoop)

    def __onMessageFunction(self, client, userdata, topic, message):
        if topic == CONST.TOPICS.DIRECTIONS:
            userdata["setDirections"](message)
        elif topic == CONST.TOPICS.VEHICLE_STATUS:
            userdata["setOtherVehiclePoses"](message)
        elif topic == CONST.TOPICS.TRAFFICSIGNAL_STATUS:
            userdata["setTrafficSignals"](message)

    def publishVehicleStatus(self):
        message = json.dumps({
            "vehicleID": self.vehicleID,
            "lat": self.lat,
            "lng": self.lng,
            "heading": self.heading,
            "waypointID": self.waypointID,
            "arrowID": self.arrowID,
            "schedule": self.schedules[0] if 0 < len(self.schedules) else None
        })
        self.publish(CONST.TOPICS.VEHICLE_STATUS, message)

    def setDirections(self, message):
        directions = json.loads(message)
        pp(["setDirections", directions])
        if self.vehicleID == directions["vehicleID"]:
            self.schedules = directions["schedules"]

    def setTrafficSignals(self, message):
        trafficSignalStatus = json.loads(message)
        pp(["setTrafficSignals", trafficSignalStatus])
        # todo: localize
        entryExitRouteID = trafficSignalStatus["entryExitRouteID"]
        self.trafficSignals[entryExitRouteID] = trafficSignalStatus

    def setOtherVehiclePoses(self, message):
        vehicleStatus = json.loads(message)
        if vehicleStatus["vehicleID"] != self.vehicleID:
            # todo: localize
            self.otherVehicles[vehicleStatus["vehicleID"]] = vehicleStatus

    def getMonitoredRoute(self, distance=100.0):
        if distance <= 0:
            return None
        arrowIDs = self.schedules[0]["content"]["route"]["arrowIDs"]
        arrowIDs = arrowIDs[arrowIDs.index(self.arrowID):]
        route = {
            "startWaypointID": self.waypointID,
            "goalWaypointID": self.arrow.getWaypointIds(self.schedules[0]["content"]["route"]["arrowIDs"][-1])[-1],
            "arrowIDs": arrowIDs
        }
        return self.route.getSlicedRoute(route, distance)

    def __getInterVehicleDistance(self, monitoredRoute):
        monitoredWaypointIDs = self.route.getRouteWaypointIDs(monitoredRoute)
        interVehicleDistance = CONST.FLOAT_MAX
        if self.arrowID is not None and 0 < len(self.otherVehicles):
            otherVehiclesWaypointIDs = list(map(lambda x: x["waypointID"], self.otherVehicles.values()))
            for i, monitoredWaypointID in enumerate(monitoredWaypointIDs):
                if monitoredWaypointID in otherVehiclesWaypointIDs:
                    interVehicleDistance = self.route.getDistanceOfWaypoints(monitoredWaypointIDs[0:i + 1])
                    break
        print("interVehicleDistance {}[m]".format(interVehicleDistance))
        return interVehicleDistance

    def __getInterTrafficSignalDistance(self, monitoredRoute):
        routeArrowIDs = monitoredRoute["arrowIDs"]
        interTrafficSignalDistance = CONST.FLOAT_MAX

        intersectionIDs = list(set(map(lambda x: x["intersectionID"], self.trafficSignals.values())))
        entryExitRouteIDs = []
        for intersectionID in intersectionIDs:
            entryExitRouteIDs.extend(
                self.intersection.getEntryExitRouteIDsInArrowIDs(intersectionID, routeArrowIDs))

        minArrowIDIndex = len(routeArrowIDs)
        minGoalWaypointID = None
        for entryExitRouteID in entryExitRouteIDs:
            if self.trafficSignals[entryExitRouteID]["state"] in [CONST.TRAFFICSIGNAL_STATE.YELLOW, CONST.TRAFFICSIGNAL_STATE.RED]:
                entryExitRouteArrowIDs = self.intersection.getArrowIDsOfEntryExitRoute(intersectionID, entryExitRouteID)
                entryArrowID = entryExitRouteArrowIDs[0]
                print(self.trafficSignals[entryExitRouteID]["state"], entryArrowID)

                entryArrowIDIndex = routeArrowIDs.index(entryArrowID)
                if entryArrowIDIndex < minArrowIDIndex:
                    minArrowIDIndex = entryArrowIDIndex
                    borderPoint = self.intersection.getBorderPoint(intersectionID, entryArrowID)
                    minGoalWaypointID = borderPoint["prevWaypointID"]

        if minGoalWaypointID is not None:
            minRoute = {
                "startWaypointID": monitoredRoute["startWaypointID"],
                "goalWaypointID": minGoalWaypointID,
                "arrowIDs": routeArrowIDs[:minArrowIDIndex+1],
            }
            interTrafficSignalDistance = self.route.getRouteLength(minRoute)

        print("interTrafficSignalDistance {}[m]".format(interTrafficSignalDistance))
        return interTrafficSignalDistance

    def __getMovableDistance(self):
        movableDistance = CONST.FLOAT_MAX
        if 0 < len(self.schedules):
            if "move" == self.schedules[0]["content"]["type"]:
                # check inter-vehicle distance
                monitoredRoute = self.getMonitoredRoute()
                if monitoredRoute is None:
                    return 0.0
                interVehicleDistance = self.__getInterVehicleDistance(monitoredRoute)
                movableDistance = interVehicleDistance - LOWER_INTER_VEHICLE_DISTANCE

                # check inter-trafficSignal distance
                monitoredRoute = self.getMonitoredRoute(movableDistance)
                if monitoredRoute is None:
                    return 0.0
                interTrafficSignalDistance = self.__getInterTrafficSignalDistance(monitoredRoute)
                movableDistance = min(movableDistance, interTrafficSignalDistance - LOWER_INTER_TRAFFIC_SIGNAL_DISTANCE)

        return movableDistance

    def updateState(self, dt=1):
        if 0 < len(self.schedules):
            if "move" == self.schedules[0]["content"]["type"]:
                prevWaypointID = self.waypointID
                goalWaypointID = self.schedules[0]["content"]["route"]["goalWaypointID"]
                deltaDistance = min(self.velocity * dt, self.__getMovableDistance())
                if 0.0 < deltaDistance:
                    self.lat, self.lng, self.heading, self.arrowID, self.waypointID = self.getNextPose(deltaDistance)

                    # get pass waypointIDs
                    waypointIDs = []
                    for arrowID in self.schedules[0]["content"]["route"]["arrowIDs"][0:2]:
                        waypointIDs.extend(self.arrow.getWaypointIds(arrowID))
                    passWaypointIDs = waypointIDs[waypointIDs.index(prevWaypointID):waypointIDs.index(self.waypointID)+1]
                    # print("updateState", [prevWaypointID], [self.waypointID], passWaypointIDs, [goalWaypointID], goalWaypointID in passWaypointIDs)

                    # if self.waypointID == self.schedules[0]["content"]["route"]["goalWaypointID"]:
                    if goalWaypointID in passWaypointIDs:
                        print("*** arrival ***")
                        self.waypointID = goalWaypointID
                        self.lat, self.lng = self.waypoint.getLatLng(self.waypointID)
                        self.heading = self.arrow.getHeading(self.arrowID, self.waypointID)
                        self.schedules.pop(0)

                        # update next schedule
                        newStartTime = time()
                        difTime = newStartTime - self.schedules[0]["startTime"]
                        self.schedules[0]["startTime"] += difTime
                        self.schedules[0]["endTime"] += difTime
                    else:
                        arrowIDs = self.schedules[0]["content"]["route"]["arrowIDs"]
                        self.schedules[0]["content"]["route"]["arrowIDs"] = arrowIDs[arrowIDs.index(self.arrowID):]

            elif "stop" == self.schedules[0]["content"]["type"]:
                currentTime = time()
                if self.schedules[0]["endTime"] < currentTime:
                    self.schedules.pop(0)

                    # update next schedule
                    difTime = currentTime - self.schedules[0]["startTime"]
                    self.schedules[0]["startTime"] += difTime
                    self.schedules[0]["endTime"] += difTime

            elif "standBy" == self.schedules[0]["content"]["type"]:
                if 1 < len(self.schedules):
                    currentTime = time()
                    self.schedules.pop(0)

                    # update next schedule
                    difTime = currentTime - self.schedules[0]["startTime"]
                    self.schedules[0]["startTime"] += difTime
                    self.schedules[0]["endTime"] += difTime

        self.publishVehicleStatus()

    def getNextPose(self, deltaDistance):
        arrows, toArrows, _ = self.arrow.getArrowIDsToArrows(
            self.schedules[0]["content"]["route"]["arrowIDs"][0:2])
        lat, lng, arrowID = self.arrow.getAdvancedLatLngInArrows(self.lat, self.lng, deltaDistance, arrows=arrows, nextArrows=toArrows)
        waypointID, lat, lng, _ = self.arrow.getPointToArrow(lat, lng, arrowID)
        heading = self.arrow.getHeading(arrowID, waypointID)
        return lat, lng, heading, arrowID, waypointID

    def __mainLoop(self):
        sleep(1)

        self.publishVehicleStatus()

        while True:
            if self.schedules is not None:
                self.updateState(self.dt)
                sleep(self.dt)

        return True


if __name__ == '__main__':
    import sys
    sys.argv[1]

    startWaypointID = "9566"  # 9232
    waypoint = Waypoint()
    waypoint.load("../res/waypoint.json")
    lat, lng = waypoint.getLatLng(startWaypointID)

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

    """
    import random
    arrow = Arrow()
    arrow.load("./res/arrow.json")
    waypoint = Waypoint()
    waypoint.load("./res/waypoint.json")
    from route import Route
    route = Route()
    route.setWaypoint(waypoint)
    route.setArrow(arrow)
    nextStartWaypointID = startWaypointID
    for i in range(10):
        startPoint = {
            "arrowID": arrow.getArrowIDsFromWaypointID(nextStartWaypointID)[0],
            "waypointID": nextStartWaypointID,
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
        if goalWaypointID == nextStartWaypointID:
            continue

        goalID = "route" + goalWaypointID
        goalPoints = [{
            "goalID": goalID,
            "arrowID": arrow.getArrowIDsFromWaypointID(goalWaypointID)[0],
            "waypointID": goalWaypointID,
        }]
        pp([startPoint, goalPoints])

        routes = route.getShortestRoutes(startPoint, goalPoints, reverse=False)
        pp(routes)

        schedules.append({
            "scheduleID": "run_" + str(i),
            "startTime": 1,
            "endTime": 2,
            "content": {
                "type": CONST.VEHICLE_STATE.MOVE,
                "route": {
                    "goalWaypointID": routes[goalID]["goalWaypointID"],
                    "arrowIDs": routes[goalID]["arrowIDs"]
                }
            }
        })
        nextStartWaypointID = routes[goalID]["goalWaypointID"]

    schedules.append(
        {
            "scheduleID": "end",
            "startTime": -1,
            "endTime": -1,
            "content": {
                "type": "standBy",
            }
        },
    )
    # """

    waypoint = Waypoint()
    waypoint.load("../res/waypoint.json")

    arrow = Arrow(waypoint)
    arrow.load("../res/arrow.json")

    route = Route()
    route.setWaypoint(waypoint)
    route.setArrow(arrow)

    intersection = Intersection()
    intersection.load("../res/intersection.json")

    vehicle = Vehicle(
        vehicleID=sys.argv[1],
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        intersection=intersection,
        waypointID=startWaypointID,
        velocity=0.00003333,
        schedules=schedules,
        dt=0.5
    )
    vehicle.start()
