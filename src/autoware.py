#!/usr/bin/env python
# coding: utf-8

from vehicle import Vehicle
from arrow import Arrow
from transforms3d.quaternions import axangle2quat
import json
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class Autoware(Vehicle):
    def __init__(self, vehicleID, waypoint, arrow, route, intersection, waypointID, velocity, numOfPassengers=0, schedules=None, dt=1.0):
        super().__init__(vehicleID, waypoint, arrow, route, intersection, waypointID, velocity, numOfPassengers, schedules, dt)

        self.setOnMessageFunction(
            onMessageFunction=self.__onMessageFunction,
            userData={
                "setDirections": self.setDirections,
                "setAutowarePose": self.setAutowarePose,
                "setOtherVehiclePoses": self.setOtherVehiclePoses,
                "setTrafficSignals": self.setTrafficSignals,
            }
        )
        self.setSubscriber(CONST.TOPICS.DIRECTIONS)
        self.setSubscriber(CONST.TOPICS.AUTOWARE.CLOSEST_WAYPOINT)
        self.setSubscriber(CONST.TOPICS.VEHICLE_STATUS)
        self.setSubscriber(CONST.TOPICS.TRAFFICSIGNAL_STATUS)

        self.__poseIndex = 0
        self.__currentPoses = []

    def __onMessageFunction(self, client, userdata, topic, message):
        if topic == CONST.TOPICS.DIRECTIONS:
            userdata["setDirections"](message)
        elif topic == CONST.TOPICS.AUTOWARE.CLOSEST_WAYPOINT:
            userdata["setAutowarePose"](message)
        elif topic == CONST.TOPICS.VEHICLE_STATUS:
            userdata["setOtherVehiclePoses"](message)
        elif topic == CONST.TOPICS.TRAFFICSIGNAL_STATUS:
            userdata["setTrafficSignals"](message)

    def setAutowarePose(self, message):
        messageDict = json.loads(message)
        if 0 <= messageDict["index"] < len(self.__currentPoses):
            self.__poseIndex = messageDict["index"]
            print(self.__currentPoses[self.__poseIndex])
            self.arrowID = self.__currentPoses[self.__poseIndex]["arrowID"]
            self.waypointID = self.__currentPoses[self.__poseIndex]["waypointID"]
            self.lat, self.lng = self.waypoint.getLatLng(self.waypointID)
            self.heading = self.arrow.getHeading(self.arrowID, self.waypointID)
        else:
            print("Lost Autoware.")

    def updateState(self, dt=1):
        print("updateState", len(self.schedules))
        if 0 < len(self.schedules):
            if "move" == self.schedules[0]["content"]["type"]:
                if len(self.__currentPoses) == 0:
                    self.__sendWaypointsToAutoware()
                else:
                    if self.__poseIndex == len(self.__currentPoses)-2:
                        print("*** arrival ***")
                        self.publish(CONST.TOPICS.NOTICE, ",".join(map(str, [self.vehicleID, self.waypointID])))
                        self.schedules.pop(0)

                        # update next schedule
                        newStartTime = time()
                        difTime = newStartTime - self.schedules[0]["startTime"]
                        self.schedules[0]["startTime"] += difTime
                        self.schedules[0]["endTime"] += difTime

            elif "stop" == self.schedules[0]["content"]["type"]:
                currentTime = time()
                if self.schedules[0]["endTime"] < currentTime:
                    self.schedules.pop(0)

                    # update next schedule
                    difTime = currentTime - self.schedules[0]["startTime"]
                    self.schedules[0]["startTime"] += difTime
                    self.schedules[0]["endTime"] += difTime

                    self.__currentPoses = []

            elif "standBy" == self.schedules[0]["content"]["type"]:
                currentTime = time()
                print(self.schedules[0]["endTime"], currentTime)
                if self.schedules[0]["endTime"] < currentTime:
                    if 1 < len(self.schedules):
                        self.schedules.pop(0)

                        # update next schedule
                        difTime = currentTime - self.schedules[0]["startTime"]
                        self.schedules[0]["startTime"] += difTime
                        self.schedules[0]["endTime"] += difTime

                        self.__currentPoses = []

        self.publishVehicleStatus()

    def __sendWaypointsToAutoware(self):
        print("__sendWaypointsToAutoware")
        waypoints = []
        arrowWaypointArray = []
        schedule = self.schedules[0]
        if schedule["content"]["type"] == CONST.VEHICLE_STATE.STANDBY:
            pass
        elif schedule["content"]["type"] == CONST.VEHICLE_STATE.MOVE:
            arrowWaypointArray = self.route.getArrowWaypointArray(schedule["content"]["route"])
            for arrowWaypoint in arrowWaypointArray:
                waypointID = arrowWaypoint["waypointID"]
                arrowID = arrowWaypoint["arrowID"]
                waypoint = self.waypoint.getWaypoint(waypointID)
                w, x, y, z = axangle2quat([0, 0, 1], waypoint["yaw"])
                waypoints.append({
                    "position": {
                        "x": waypoint["x"],
                        "y": waypoint["y"],
                        "z": waypoint["z"],
                    },
                    "orientation": {
                        "w": w,
                        "x": x,
                        "y": y,
                        "z": z,
                    },
                    "velocity": 2.0
                })
        if (0 < len(waypoints)):
            num = min(15, len(waypoints))
            for i in range(num-1, 0, -1):
                waypoints[-i]["velocity"] = ((i)/num)*waypoints[-i-1]["velocity"]
            self.__currentPoses = arrowWaypointArray
            message = json.dumps(waypoints)
            self.publish(CONST.TOPICS.AUTOWARE.LANES, message)


if __name__ == "__main__":
    import sys
    from waypoint import Waypoint
    from route import Route
    from intersection import Intersection
    from const import CONST

    sys.argv[1]

    startWaypointID = "9566"  #"8809"  # "9566"  # 9232
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
    arrow.load("../res/arrow.json")
    waypoint = Waypoint()
    waypoint.load("../res/waypoint.json")
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
        # pp([startPoint, goalPoints])

        routes = route.getShortestRoutes(startPoint, goalPoints, reverse=False)
        # pp(routes)

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

    autoware = Autoware(
        vehicleID=sys.argv[1],
        waypoint=waypoint,
        arrow=arrow,
        route=route,
        intersection=intersection,
        waypointID=startWaypointID,
        velocity=0.00001,
        schedules=schedules,
        dt=0.5
    )
    autoware.start()
