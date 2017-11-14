#!/usr/bin/env python
# coding: utf-8
import json


class Intersection(object):
    def load(self, path):
        with open(path, "r") as f:
            data = json.load(f)
            self.setIntersections(data["intersections"])
        return True

    def connectToRedis(self, host, port, dbname):
        return None

    def setIntersections(self, intersections):
        self.__intersections = intersections

    def getIntersections(self):
        return self.__intersections

    def getIntersection(self, intersectionID):
        return self.__intersections[intersectionID]

    def getIntersectionIDs(self):
        return list(self.__intersections.keys())

    def getEntryExitRouteIDs(self, intersectionID):
        return list(self.__intersections[intersectionID]["entryExitRoutes"].keys())

    def getBorderPoint(self, intersectionID, arrowID):
        return list(filter(
            lambda x: x["arrowID"] == arrowID,
            self.__intersections[intersectionID]["borderPoints"]))[0]

    def getToInBorderWaypointIDs(self, intersectionID):
        return list(map(
            lambda x: x["prevWaypointID"],
            filter(lambda x: x["toIn"], self.__intersections[intersectionID]["borderPoints"])))

    def getToInArrowIDs(self, intersectionID):
        return list(map(
            lambda x: x["arrowID"],
            filter(lambda x: x["toIn"], self.__intersections[intersectionID]["borderPoints"])))

    def getEntryExitRouteArrowIDsSet(self, intersectionID):
        return list(map(
            lambda x: (x[0], x[1]["arrowIDs"]),
            self.__intersections[intersectionID]["entryExitRoutes"].items()))

    def getEntryExitRouteIDsInArrowIDs(self, intersectionID, arrowIDs):
        entryExitRouteIDs = []
        toInArrowIDs = self.getToInArrowIDs(intersectionID)
        for i, arrowID in enumerate(arrowIDs):
            if arrowID in toInArrowIDs:
                for entryExitRouteID, entryExitRoute in self.__intersections[intersectionID]["entryExitRoutes"].items():
                    entryExitRouteArrowIDs = entryExitRoute["arrowIDs"]
                    length = len(entryExitRouteArrowIDs)
                    if arrowIDs[i:i+length] == entryExitRouteArrowIDs:
                        entryExitRouteIDs.append(entryExitRouteID)
        return entryExitRouteIDs

    def getArrowIDsOfEntryExitRoute(self, intersectionID, entryExitRouteID):
        return self.__intersections[intersectionID]["entryExitRoutes"][entryExitRouteID]["arrowIDs"]



if __name__ == '__main__':
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2).pprint

    intersection = Intersection()
    intersection.load("./res/intersection.json")

    intersectionIDs = intersection.getIntersectionIDs()
    intersections = intersection.getIntersections()
    pp(list(map(lambda x: (intersections[x]["lat"], intersections[x]["lng"]), intersectionIDs)))

    # with open("./intersectionsLatLng.json", "w") as f:
    #     json.dump(list(map(lambda x: (intersections[x]["lat"], intersections[x]["lng"]), intersectionIDs)), f, indent="  ")
