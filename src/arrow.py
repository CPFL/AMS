#!/usr/bin/env python
# coding: utf-8
import math
import json
from sys import float_info


# import sys
# sys.path.append("/Users/yashiro/workspace/tools")
# from debugger import dumpInputOnFunctionException


class Arrow(object):
    def __init__(self, waypoint=None):
        self.waypoint = waypoint

    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.setArrows(data["arrows"], data["toArrows"], data["fromArrows"])
        return True

    def connectToRedis(self, host, port, dbname):
        return None

    def setArrows(self, arrows, toArrows, fromArrows):
        self.__arrows = arrows
        self.__toArrows = toArrows
        self.__fromArrows = fromArrows

    def getArrows(self):
        return self.__arrows

    def getToArrows(self):
        return self.__toArrows

    def getFromArrows(self):
        return self.__fromArrows

    def getWaypointIds(self, arrowID):
        return self.__arrows[arrowID]["waypointIDs"]

    def getApproximateDistance(self, lat1, lng1, lat2, lng2):
        distance = math.hypot(lat1-lat2, lng1-lng2)
        return distance

    def getHeading(self, arrowID, waypointID):
        waypointIDs = self.__arrows[arrowID]["waypointIDs"]
        index = waypointIDs.index(waypointID)
        lat1, lng1 = self.waypoint.getLatLng(waypointIDs[max([0, index - 1])])
        lat2, lng2 = self.waypoint.getLatLng(waypointIDs[min([len(waypointIDs) - 1, index + 1])])
        return 180+math.degrees(math.atan2(lng1 - lng2, lat1 - lat2))

    def getArrowHeading(self, arrowID):
        """
        arrowの方角(deg)取得。
        :param arrowID:
        :return:
        """
        return 180+math.degrees(math.atan2(
            self.__arrows[arrowID]["lng1"] - self.__arrows[arrowID]["lng2"],
            self.__arrows[arrowID]["lat1"] - self.__arrows[arrowID]["lat2"]
        ))

    def getPointToEdge(self, pointLat, pointLng, edgeLat1, edgeLng1, edgeLat2, edgeLng2):
        lat12 = edgeLat2 - edgeLat1
        lng12 = edgeLng2 - edgeLng1
        lat1p = pointLat - edgeLat1
        lng1p = pointLng - edgeLng1

        # get unit vector
        len12 = math.hypot(lat12, lng12)
        uLat12 = lat12 / len12
        uLng12 = lng12 / len12

        # dot product
        distance1X = uLat12 * lat1p + uLng12 * lng1p
        if len12 < distance1X:
            return edgeLat2, edgeLng2, self.getApproximateDistance(pointLat, pointLng, edgeLat2, edgeLng2)
        elif distance1X < 0.0:
            return edgeLat1, edgeLng1, self.getApproximateDistance(pointLat, pointLng, edgeLat1, edgeLng1)
        else:
            lat = edgeLat1 + (uLat12 * distance1X)
            lng = edgeLng1 + (uLng12 * distance1X)
            return lat, lng, self.getApproximateDistance(pointLat, pointLng, lat, lng)

    def getPointToArrow(self, pointLat, pointLng, arrowID):
        matchedWaypoints = {}
        prevLat, prevLng = None, None
        for waypointID in self.__arrows[arrowID]["waypointIDs"]:
            lat, lng = self.waypoint.getLatLng(waypointID)
            if not None in [prevLat, prevLng]:
                latOnEdge, lngOnEdge, distance = self.getPointToEdge(pointLat, pointLng, prevLat, prevLng, lat, lng)
                matchedWaypoints[waypointID] = {
                    "waypointID": waypointID,
                    "lat": latOnEdge,
                    "lng": lngOnEdge,
                    "distance": distance
                }
            prevLat, prevLng = lat, lng

        if len(matchedWaypoints) == 0:
            raise Exception

        mostMatchedWaypoint = min(matchedWaypoints.values(), key=lambda x: x["distance"])
        return mostMatchedWaypoint["waypointID"], mostMatchedWaypoint["lat"], mostMatchedWaypoint["lng"], mostMatchedWaypoint["distance"]

    def getPointToArrows(self, pointLat, pointLng, arrows=None):
        matchedWaypoints = {}
        if arrows is None:
            arrows = self.__arrows
        for arrowID in arrows:
            waypointID, lat, lng, distance = self.getPointToArrow(pointLat, pointLng, arrowID)
            matchedWaypoints[waypointID] = {
                "arrowID": arrowID,
                "waypointID": waypointID,
                "lat": lat,
                "lng": lng,
                "distance": distance
            }

        if len(matchedWaypoints) == 0:
            raise Exception

        mostMatchedWaypoint = min(matchedWaypoints.values(), key=lambda x: x["distance"])
        return mostMatchedWaypoint["arrowID"], mostMatchedWaypoint["waypointID"], mostMatchedWaypoint["lat"], mostMatchedWaypoint["lng"], mostMatchedWaypoint["distance"]

    def getAdvancedLatLngInArrow(self, lat, lng, deltaDistance, arrowID, nextWaypointID=None):
        if nextWaypointID is None:
            nextWaypointID, latOnEdge, lngOnEdge, _ = self.getPointToArrow(lat, lng, arrowID)
        else:
            latOnEdge, lngOnEdge = lat, lng

        nextLat, nextLng = self.waypoint.getLatLng(nextWaypointID)
        remainingDistance = self.getApproximateDistance(latOnEdge, lngOnEdge, nextLat, nextLng)
        if deltaDistance <= remainingDistance:
            lat12 = nextLat - latOnEdge
            lng12 = nextLng - lngOnEdge
            len12 = math.hypot(lat12, lng12)
            uLat12 = lat12 / len12
            uLng12 = lng12 / len12
            advancedLat = latOnEdge + (uLat12 * deltaDistance)
            advancedLng = lngOnEdge + (uLng12 * deltaDistance)
            return advancedLat, advancedLng, 0.0
        else:
            waypointIDs = self.__arrows[arrowID]["waypointIDs"]
            index = waypointIDs.index(nextWaypointID) + 1
            if index == len(waypointIDs):
                return nextLat, nextLng, deltaDistance - remainingDistance
            return self.getAdvancedLatLngInArrow(nextLat, nextLng, deltaDistance - remainingDistance, arrowID, waypointIDs[index])

    def getAdvancedLatLngInArrows(self, lat, lng, deltaDistance, arrows, nextArrows):
        arrowID, _, _, _, _ = self.getPointToArrows(lat, lng, arrows)
        remainingDistance = deltaDistance
        advancedLat, advancedLng = lat, lng
        while True:
            advancedLat, advancedLng, remainingDistance = self.getAdvancedLatLngInArrow(advancedLat, advancedLng, remainingDistance, arrowID)
            if remainingDistance == 0 or arrowID not in nextArrows:
                return advancedLat, advancedLng, arrowID
            arrowID = nextArrows[arrowID][0]

    def getNextLatLng(self, lat, lng, deltaDistance, arrowID=None, arrows=None, toArrows=None):
        if arrows is None:
            arrows = self.__arrows
        if toArrows is None:
            toArrows = self.__toArrows
        latOnArrow, lngOnArrow = lat, lng
        if arrowID is None:
            arrowID, _, latOnArrow, lngOnArrow, _ = self.getPointToArrows(lat, lng, arrows=arrows)
        remainingDistance = self.getApproximateDistance(latOnArrow, lngOnArrow, arrows[arrowID]["lat2"], arrows[arrowID]["lng2"])
        if deltaDistance <= remainingDistance:
            lat12 = arrows[arrowID]["lat2"] - latOnArrow
            lng12 = arrows[arrowID]["lng2"] - lngOnArrow
            len12 = math.hypot(lat12, lng12)
            uLat12 = lat12 / len12
            uLng12 = lng12 / len12
            advancedLat = latOnArrow + (uLat12 * deltaDistance)
            advancedLng = lngOnArrow + (uLng12 * deltaDistance)
            return advancedLat, advancedLng, mostMatchedWaypointID
        else:
            nextArrow = arrows[toArrows[arrowID][0]]
            return self.getNextLatLng(nextArrow["lat1"], nextArrow["lng1"], deltaDistance - remainingDistance, nextArrow["name"], arrows=arrows, toArrows=toArrows)

    def getAdvancedLatLng(self, lat, lng, deltaDistance, arrowID=None, arrows=None, toArrows=None):
        print("getAdvancedLatLng", arrows.keys(), toArrows.keys())
        if arrows is None:
            arrows = self.__arrows
        if toArrows is None:
            toArrows = self.__toArrows
        latOnArrow, lngOnArrow = lat, lng
        if arrowID is None:
            arrowID, latOnArrow, lngOnArrow = self.getPointToArrows(lat, lng, arrows=arrows)
        remainingDistance = self.getApproximateDistance(latOnArrow, lngOnArrow, arrows[arrowID]["lat2"], arrows[arrowID]["lng2"])
        if deltaDistance <= remainingDistance:
            lat12 = arrows[arrowID]["lat2"] - latOnArrow
            lng12 = arrows[arrowID]["lng2"] - lngOnArrow
            len12 = math.hypot(lat12, lng12)
            uLat12 = lat12 / len12
            uLng12 = lng12 / len12
            advancedLat = latOnArrow + (uLat12 * deltaDistance)
            advancedLng = lngOnArrow + (uLng12 * deltaDistance)
            return advancedLat, advancedLng
        else:
            nextArrow = arrows[toArrows[arrowID][0]]
            return self.getAdvancedLatLng(nextArrow["lat1"], nextArrow["lng1"], deltaDistance - remainingDistance, nextArrow["name"], arrows=arrows, toArrows=toArrows)

    def getWaypointsNames(self, arrowID):
        return arrowID.split("/")

    def getArrowIDsToArrows(self, arrowIDs):
        arrows = {}
        toArrows = {}
        fromArrows = {}
        for arrowID in arrowIDs:
            arrows[arrowID] = self.__arrows[arrowID]

        for i in range(1, len(arrowIDs)):
            toArrows[arrowIDs[i - 1]] = [arrowIDs[i]]
            fromArrows[arrowIDs[i]] = [arrowIDs[i - 1]]

        return arrows, toArrows, fromArrows

    def getArrowIDsFromWaypointID(self, waypointID):
        return list(map(lambda x: x[0], filter(lambda x: waypointID in x[1]["waypointIDs"], self.__arrows.items())))

    def getArrowIDsSetFromWaypointIDs(self, waypointIDs):
        arrowIDsSet = [[]]
        for waypointID in waypointIDs:
            arrowIDs = self.getArrowIDsFromWaypointID(waypointID)
            for arrowID in arrowIDs:
                if self.__arrows[arrowID]["waypointIDs"][-1] in waypointIDs:
                    for i in range(len(arrowIDsSet)):
                        arrowIDsSet[i].append(arrowID)
        return arrowIDsSet


if __name__ == '__main__':
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2).pprint

    arrow = Arrow()
    arrow.load("./res/arrow.json")
    # pp(arrow.getArrows())

    # pp(arrows.getPointToArrows("35.647441", "139.772060"))

    # pp(arrows.getPointToArrows(35.65121944507907, 139.77862046921132))
    #
    # pp(arrows.getShortestRoutes(
    #     35.6504049, 139.7772154,
    #     [{'lat': 35.650421105409954,
    #     'lng': 139.7772497249228,
    #     'vehicleID': 'vehicle1'}],
    #     reverse=True))
