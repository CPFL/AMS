#!/usr/bin/env python
# coding: utf-8
from math import hypot
import json
from sys import float_info
from const import CONST


class Route(object):
    def __init__(self):
        self.__getRouteCost = self.getRouteLength

    def setWaypoint(self, waypoint):
        self.__waypoint = waypoint
        self.__waypoints = waypoint.getWaypoints()

    def setArrow(self, arrow):
        self.__arrow = arrow
        self.__arrows = arrow.getArrows()

    def setCostFunction(self, costFunction):
        """
        costFunction(arg = route{"startWaypointID": X, "goalWaypointID": X, "arrowIDs": X})
        :param costFunction: 
        :return: 
        """
        self.__getRouteCost = costFunction

    def load(self, path):
        with open(path, "r") as f:
            self.__fixedRoutes = json.load(f)
        return True

    def getFixedRoute(self, routeName):
        return self.__fixedRoutes[routeName]

    def __arrowIDToRoute(self, arrowID):
        arrow = self.__arrows[arrowID]
        return {
            "startWaypointID": arrow["waypointIDs"][0],
            "goalWaypointID": arrow["waypointIDs"][-1],
            "arrowIDs": [arrowID]
        }

    def getArrowWaypointArray(self, route):
        arrowIDs = route["arrowIDs"]
        startWaypointID = route["startWaypointID"]
        goalWaypointID = route["goalWaypointID"]
        arrowWaypointArray = []
        for arrowID in arrowIDs:
            waypointIDs = self.__arrows[arrowID]["waypointIDs"]
            js = 0
            if startWaypointID in waypointIDs:
                js = waypointIDs.index(startWaypointID)
            je = len(waypointIDs)
            if goalWaypointID in waypointIDs:
                je = waypointIDs.index(goalWaypointID) + 1
            for i in range(js+1, je):
                arrowWaypointArray.append({"waypointID": waypointIDs[i - 1], "arrowID": arrowID})
            if arrowID == arrowIDs[-1]:
                arrowWaypointArray.append({"waypointID": waypointIDs[je - 1], "arrowID": arrowID})
        return arrowWaypointArray

    def getRouteWaypointIDs(self, route):
        arrowIDs = route["arrowIDs"]
        startWaypointID = route["startWaypointID"]
        goalWaypointID = route["goalWaypointID"]
        routeWaypointIds = []
        for arrowID in arrowIDs:
            waypointIDs = self.__arrows[arrowID]["waypointIDs"]
            js = 0
            if startWaypointID in waypointIDs:
                js = waypointIDs.index(startWaypointID)
            je = len(waypointIDs)
            if goalWaypointID in waypointIDs:
                je = waypointIDs.index(goalWaypointID) + 1
            for i in range(js+1, je):
                routeWaypointIds.append(waypointIDs[i - 1])
            if arrowID == arrowIDs[-1]:
                routeWaypointIds.append(waypointIDs[je - 1])
        return routeWaypointIds

    def getRouteLength(self, route):
        arrowIDs = route["arrowIDs"]
        startWaypointID = route["startWaypointID"]
        goalWaypointID = route["goalWaypointID"]
        length = 0.0
        for arrowID in arrowIDs:
            waypointIDs = self.__arrows[arrowID]["waypointIDs"]
            js = 0
            if startWaypointID in waypointIDs:
                js = waypointIDs.index(startWaypointID)
            je = len(waypointIDs)
            if goalWaypointID in waypointIDs:
                je = waypointIDs.index(goalWaypointID) + 1

            if js == 0 and je == len(waypointIDs):
                length += self.__arrows[arrowID]["length"]
            else:
                for i in range(js+1, je):
                    waypoint1 = self.__waypoints[waypointIDs[i - 1]]
                    waypoint2 = self.__waypoints[waypointIDs[i]]
                    length += CONST.METER_PER_BL * hypot(waypoint1["lat"] - waypoint2["lat"], waypoint1["lng"] - waypoint2["lng"])
        return length

    def __isDirectlyReach(self, arrowID, startWaypointID, goalWaypointID, reverse):
        waypointIDs = self.__arrows[arrowID]["waypointIDs"]
        isDirectlyReach = waypointIDs.index(startWaypointID) <= waypointIDs.index(goalWaypointID)
        return isDirectlyReach if not reverse else not isDirectlyReach

    def getDistanceOfWaypoints(self, waypointIDs):
        distance = 0.0
        for i in range(1, len(waypointIDs)):
            distance += CONST.METER_PER_BL * hypot(
                self.__waypoints[waypointIDs[i]]["lat"] - self.__waypoints[waypointIDs[i - 1]]["lat"],
                self.__waypoints[waypointIDs[i]]["lng"] - self.__waypoints[waypointIDs[i - 1]]["lng"]
            )
        return distance

    def getSlicedRoute(self, route, length):
        arrowIDs = route["arrowIDs"]
        startWaypointID = route["startWaypointID"]
        goalWaypointID = route["goalWaypointID"]
        totalLength = 0.0
        slicedGoalWaypointID = startWaypointID
        slicedArrowIDs = []
        for arrowID in arrowIDs:
            waypointIDs = self.__arrows[arrowID]["waypointIDs"]
            js = 0
            if startWaypointID in waypointIDs:
                js = waypointIDs.index(startWaypointID)
            je = len(waypointIDs)
            if goalWaypointID in waypointIDs:
                je = waypointIDs.index(goalWaypointID) + 1

            for i in range(js+1, je):
                totalLength += CONST.METER_PER_BL * hypot(
                    self.__waypoints[waypointIDs[i]]["lat"]-self.__waypoints[waypointIDs[i-1]]["lat"],
                    self.__waypoints[waypointIDs[i]]["lng"]-self.__waypoints[waypointIDs[i-1]]["lng"])
                if length <= totalLength:
                    return {
                        "startWaypointID": startWaypointID,
                        "goalWaypointID": slicedGoalWaypointID,
                        "arrowIDs": slicedArrowIDs
                    }
                if len(slicedArrowIDs) == 0 or slicedArrowIDs[-1] != arrowID:
                    slicedArrowIDs.append(arrowID)
                slicedGoalWaypointID = waypointIDs[i]

        return route

    def getWaypointIDsOnRoute(self, route, limitLength=None):
        waypointIDs = []
        if limitLength is None:
            return self.getRouteWaypointIDs(route)
        else:
            totalLength = 0.0
            routeWaypointIDs = self.getRouteWaypointIDs(route)
            for i in range(1, len(routeWaypointIDs)):
                waypointIDs.append(routeWaypointIDs[i-1])
                totalLength += CONST.METER_PER_BL * hypot(
                    self.__waypoints[routeWaypointIDs[i]]["lat"]-self.__waypoints[routeWaypointIDs[i-1]]["lat"],
                    self.__waypoints[routeWaypointIDs[i]]["lng"]-self.__waypoints[routeWaypointIDs[i-1]]["lng"])
                if limitLength < totalLength:
                    return waypointIDs
        return waypointIDs

    def getShortestRoutes(self, start, goals, costLimit=float_info.max, reverse=False):
        # todo: コストと距離の分離
        """
        各目的地ごとの最短距離ルートを計算
        # Dijkstra's algorithm
        """
        nextArrows = self.__arrow.getToArrows()
        waypointIDEnd = self.__arrows[start["arrowID"]]["waypointIDs"][-1]
        localStartWaypointID = start["waypointID"]
        localgoalWaypointID = waypointIDEnd
        if reverse:
            nextArrows = self.__arrow.getFromArrows()
            waypointIDEnd = self.__arrows[start["arrowID"]]["waypointIDs"][0]
            localStartWaypointID = waypointIDEnd
            localgoalWaypointID = start["waypointID"]

        costStartToEnd = self.__getRouteCost({
            "startWaypointID": localStartWaypointID,
            "goalWaypointID": localgoalWaypointID,
            "arrowIDs": [start["arrowID"]]
        })

        goalArrowCandidates = self.__getGoalArrowCandidates(goals, reverse)

        checkedArrowID = []
        currentArrowID = start["arrowID"]

        endArrows = {currentArrowID: {"cost": costStartToEnd, "prevArrows": []}}
        shortestRoutes = {}

        # check same arrow
        for goalPoints in goalArrowCandidates.values():
            for goalCandidate in goalPoints.values():
                if start["arrowID"] == goalCandidate["arrowID"]:
                    if self.__isDirectlyReach(start["arrowID"], start["waypointID"], goalCandidate["waypointID"], reverse):
                        print("both start and goal on same arrow. and reach directly")
                        localStartWaypointID = start["waypointID"]
                        localgoalWaypointID = goalCandidate["waypointID"]
                        if reverse:
                            localStartWaypointID = goalCandidate["waypointID"]
                            localgoalWaypointID = start["waypointID"]
                        costStartToGoal = self.__getRouteCost({
                            "startWaypointID": localStartWaypointID,
                            "goalWaypointID": localgoalWaypointID,
                            "arrowIDs": [start["arrowID"]]
                        })
                        shortestRoutes[goalCandidate["goalID"]] = {
                            "goalID": goalCandidate["goalID"],
                            "startWaypointID": start["waypointID"],
                            "goalWaypointID": goalCandidate["waypointID"],
                            "cost": costStartToGoal,
                            "arrowIDs": [start["arrowID"]],
                        }

        while True:
            nextArrowIDs = nextArrows[currentArrowID]

            if 0 == len(nextArrowIDs):
                endArrows[currentArrowID]["cost"] = float_info.max

                endArrowsFiltered = dict(filter(lambda x: x[1]["cost"] < costLimit, endArrows.items()))
                if len(endArrowsFiltered) == 0:
                    break
                currentArrowID = min(endArrowsFiltered.items(), key=lambda x: x[1]["cost"])[0]

                continue

            for nextArrowID in nextArrowIDs:
                if nextArrowID in checkedArrowID and nextArrowID != start["arrowID"]:
                    continue
                if nextArrowID in endArrows:
                    continue

                # update endArrows
                endArrows[nextArrowID] = {
                    "cost": endArrows[currentArrowID]["cost"] + self.__getRouteCost(self.__arrowIDToRoute(currentArrowID)),
                    "prevArrows": [currentArrowID] + endArrows[currentArrowID]["prevArrows"]
                }

                for goalCandidate in goalArrowCandidates.get(nextArrowID, {}).values():
                    if endArrows[nextArrowID]["cost"] + goalCandidate["cost"] < costLimit:
                        shortestRoute = {
                            "goalID": goalCandidate["goalID"],
                            "startWaypointID": start["waypointID"],
                            "goalWaypointID": goalCandidate["waypointID"],
                            "cost": endArrows[nextArrowID]["cost"] + goalCandidate["cost"],
                            "arrowIDs": [nextArrowID] + endArrows[nextArrowID]["prevArrows"],
                        }
                        shortestRoutes[goalCandidate["goalID"]] = shortestRoute

            endArrows.pop(currentArrowID)
            checkedArrowID.append(currentArrowID)

            if len(shortestRoutes) == len(goalArrowCandidates):
                break

            endArrowsFiltered = dict(filter(lambda x: x[1]["cost"] < costLimit, endArrows.items()))
            if len(endArrowsFiltered) == 0:
                break
            currentArrowID = min(endArrowsFiltered.items(), key=lambda x: x[1]["cost"])[0]


        # reverse arrowIDs
        if not reverse:
            for routeID in shortestRoutes:
                shortestRoutes[routeID]["arrowIDs"].reverse()

        return shortestRoutes

    def addLengthToRoutes(self, routes):
        for routeID in routes:
            routes[routeID]["length"] = self.getRouteLength(routes[routeID])
        return routes

    def __getGoalArrowCandidates(self, goals, reverse):
        goalArrowCandidates = {}
        for goal in goals:
            goalID = goal["goalID"]
            arrowID = goal["arrowID"]
            if reverse:
                endWaypointID = self.__arrows[arrowID]["waypointIDs"][-1]
            else:
                endWaypointID = self.__arrows[arrowID]["waypointIDs"][0]
            cost = self.__getRouteCost({
                "startWaypointID": endWaypointID,
                "goalWaypointID": goal["waypointID"],
                "arrowIDs": [arrowID]
            })

            goalPoints = goalArrowCandidates.get(arrowID, {})
            goalPoints[goalID] = {
                "arrowID": arrowID,
                "goalID": goalID,
                "waypointID": goal["waypointID"],
                "cost": cost,
            }
            goalArrowCandidates[arrowID] = goalPoints

        return goalArrowCandidates



from flask import Flask, send_from_directory, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)


def api_response(code=200, message={}):
    response = jsonify(message)
    response.status_code = code
    return response


@app.route('/')
def root():
    return send_from_directory(
        directory="../../tools/", filename="routeViewer.html")


@app.route("/getViewData")
def getViewData():
    from arrow import Arrow
    from waypoint import Waypoint
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2).pprint

    arrow = Arrow()
    arrow.load("./res/arrow.json")

    waypoint = Waypoint()
    waypoint.load("./res/waypoint.json")

    route = Route()
    route.setWaypoint(waypoint)
    route.setArrow(arrow)
    # route.setCostFunction()
    try:
        route.load("./res/route.json")
    except:
        print("./res/route.json isnt exist")

    # startWaypointID = "9566"
    startWaypointID = "9232"

    startPoint = {
        "arrowID": arrow.getArrowIDsFromWaypointID(startWaypointID)[0],
        "waypointID": startWaypointID,
    }

    goalPoints = []
    # for goalWaypointID in ["9567"]:
    # for goalWaypointID in ["9564"]:
    for goalWaypointID in ["9232", "10000", "10361"]:
        goalPoints.append({
            "goalID": "route"+goalWaypointID,
            "arrowID": arrow.getArrowIDsFromWaypointID(goalWaypointID)[0],
            "waypointID": goalWaypointID,
        })
    pp([startPoint, goalPoints])

    routes = route.getShortestRoutes(startPoint, goalPoints, reverse=False)
    # routes = route.getShortestRoutes(startPoint, goalPoints)
    pp(routes)


    waypoints = waypoint.getWaypoints()
    arrows = arrow.getArrows()
    latMin = min(waypoints.values(), key=lambda x: x["lat"])["lat"]
    latMax = max(waypoints.values(), key=lambda x: x["lat"])["lat"]
    lngMin = min(waypoints.values(), key=lambda x: x["lng"])["lng"]
    lngMax = max(waypoints.values(), key=lambda x: x["lng"])["lng"]

    viewData = {
        "viewPoint": {
            "lat": 0.5*(latMax + latMin),
            "lng": 0.5*(lngMax + lngMin)},
        "waypoints": waypoints,
        "arrows": arrows,
        "routes": routes
    }
    return api_response(code=200, message=viewData)


# @app.route("/dumpData")
# def dumpData():
#     pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
#     vectorMap = VectorMap()
#     vectorMap.load(pathDir)
#     vectorMap.dumpData()
#     return api_response(code=200, message={"result": "done"})



if __name__ == '__main__':
    # pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    # vectorMap = VectorMap()
    # vectorMap.load(pathDir)
    # vectorMap.dumpData()
    app.run(debug=True)
