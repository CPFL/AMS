import sys
from math import hypot
import json
from vectorMap import VectorMap
from const import CONST
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint

ARROW_NAME_SEPARATER = "/"


class MapAuthoring(object):
    def __init__(self, vectorMap):
        self.__vectorMap = vectorMap
        self.__viewData = {
            "viewPoint": {
                "lat": None,
                "lng": None
            }
        }

        self.__createWaypoints()
        self.__createArrows()
        self.__createIntersections()

        self.__getViewPoint()

    def __getShapes(self):
        import json
        with open("./shapes_plus.json", "r") as f:
            shapes = json.load(f)

        # shapes = []
        # for did in self.__vectorMap.dtlane:
        #     if self.__vectorMap.dtlane[did]["Dist"] == "0":
        #         shapes.append([])
        #     shapes[-1].append(self.__vectorMap.dtlane[did]["PID"])
        # self.saveShapes(shapes, "shapes.json")
        return shapes

    def saveShapes(self, shapes, path):
        import json
        with open(path, "w") as f:
            json.dump(shapes, f, indent="  ")

    def __createWaypoints(self):
        self.__waypoints = {}
        shapes = self.__getShapes()
        for shape in shapes:
            for pointID in shape:
                point = self.__vectorMap.point[pointID]
                self.__waypoints[pointID] = {
                    "waypointID": pointID,
                    "lat": point["lat"],
                    "lng": point["lng"],
                    "height": point["H"],
                    "x": point["Bx"],
                    "y": point["Ly"],
                }

    def __createArrows(self):
        shapes = self.__getShapes()
        endPointIDs = list(set(list(map(
            lambda x: x[0], shapes
        )) + list(map(
            lambda x: x[-1], shapes
        ))))

        self.__arrows = {}
        self.__toArrows = {}
        self.__fromArrows = {}
        for shape in shapes:
            pids = []
            for pid in shape:
                pids.append(pid)
                if pid in endPointIDs:
                    if 2 < len(pids):
                        arrowID = ARROW_NAME_SEPARATER.join(map(str, [pids[0], pids[-1]]))
                        length = 0.0
                        for i in range(1, len(pids)):
                            waypoint1 = self.__waypoints[pids[i-1]]
                            waypoint2 = self.__waypoints[pids[i]]
                            length += CONST.METER_PER_BL * hypot(waypoint1["lat"]-waypoint2["lat"], waypoint1["lng"]-waypoint2["lng"])
                        self.__arrows[arrowID] = {
                            "arrowID": arrowID,
                            "waypointIDs": pids,
                            "length": length,
                        }
                        pids = [pid]

        for arrowID in self.__arrows:
            waypointID1, waypointID2 = arrowID.split(ARROW_NAME_SEPARATER)
            arrowIDs = list(self.__arrows.keys())
            self.__toArrows[arrowID] = list(filter(
                lambda x: waypointID2 == x.split(ARROW_NAME_SEPARATER)[0] and waypointID1 != x.split(ARROW_NAME_SEPARATER)[1],
                arrowIDs))
            self.__fromArrows[arrowID] = list(filter(
                lambda x: waypointID1 == x.split(ARROW_NAME_SEPARATER)[1] and waypointID2 != x.split(ARROW_NAME_SEPARATER)[0],
                arrowIDs))

    def __createIntersections(self):
        self.__intersections = {}
        for intersection in self.__vectorMap.intersection.values():
            intersectionID = intersection["AID"]

            borderLIDs = []
            area = self.__vectorMap.area[intersectionID]
            nextLID = area["SLID"]
            while True:
                borderLIDs.append(nextLID)
                nextLID = self.__vectorMap.line[nextLID]["FLID"]
                if nextLID == area["ELID"]:
                    borderLIDs.append(nextLID)
                    break

            polygon = []
            for borderLID in borderLIDs:
                bpid = self.__vectorMap.line[borderLID]["BPID"]
                blat, blng = self.__vectorMap.point[bpid]["lat"], self.__vectorMap.point[bpid]["lng"]
                polygon.append((blat, blng))

            borderPoints = self.__getBorderPoints(borderLIDs, polygon)
            entryExitRoutes = self.__getEntryExitRoutes(borderPoints)
            if 0 < len(entryExitRoutes):
                self.__intersections[intersectionID] = {
                    "intersectionID": intersectionID,
                    "polygon": polygon,
                    "borderPoints": borderPoints,
                    "entryExitRoutes": entryExitRoutes
                }

    def __getBorderPoints(self, borderLIDs, polygon):
        import sympy.geometry as sg
        borderPoints = []
        for borderLID in borderLIDs:
            bpid = self.__vectorMap.line[borderLID]["BPID"]
            fpid = self.__vectorMap.line[borderLID]["FPID"]
            blat, blng = self.__vectorMap.point[bpid]["lat"], self.__vectorMap.point[bpid]["lng"]
            flat, flng = self.__vectorMap.point[fpid]["lat"], self.__vectorMap.point[fpid]["lng"]
            for arrowID, arrow in self.__arrows.items():
                for i in range(1, len(arrow["waypointIDs"])):
                    abpid = arrow["waypointIDs"][i - 1]
                    afpid = arrow["waypointIDs"][i]
                    ablat, ablng = self.__vectorMap.point[abpid]["lat"], self.__vectorMap.point[abpid]["lng"]
                    aflat, aflng = self.__vectorMap.point[afpid]["lat"], self.__vectorMap.point[afpid]["lng"]

                    if self.isIentersected(blat, blng, flat, flng, ablat, ablng, aflat, aflng):
                        intersection = sg.intersection(
                            sg.Segment(sg.Point(blat, blng), sg.Point(flat, flng)),
                            sg.Segment(sg.Point(ablat, ablng), sg.Point(aflat, aflng))
                        )

                        toIn = self.__pointInnerPolygon((aflat, aflng), polygon)

                        borderPoint = {
                            "arrowID": arrowID,
                            "prevWaypointID": arrow["waypointIDs"][i-1],
                            "nextWaypointID": arrow["waypointIDs"][i],
                            "toIn": toIn,
                            "lat": float(intersection[0].x),
                            "lng": float(intersection[0].y),
                        }
                        if borderPoint not in borderPoints:
                            borderPoints.append(borderPoint)
        return borderPoints

    def __getEntryExitRoutes(self, borderPoints):
        entryExitRoutes = {}
        toInArrowIDs = map(lambda x: x["arrowID"], filter(lambda x: x["toIn"], borderPoints))
        toOutArrowIDs = list(map(lambda x: x["arrowID"], filter(lambda x: not x["toIn"], borderPoints)))
        for toInArrowID in toInArrowIDs:
            arrowIDsSet = [[toInArrowID]]
            filteredRoutes = list(filter(lambda x: x[-1] not in toOutArrowIDs, arrowIDsSet))
            while len(filteredRoutes):
                for filteredRoute in filteredRoutes:
                    nextArrowIDs = self.__toArrows[filteredRoute[-1]]
                    arrowIDsSet.remove(filteredRoute)
                    for nextArrowID in nextArrowIDs:
                        arrowIDsSet.append(filteredRoute + [nextArrowID])
                filteredRoutes = list(filter(lambda x: x[-1] not in toOutArrowIDs, arrowIDsSet))

            for arrowIDs in arrowIDsSet:
                entryExitRouteID = "/".join(list(map(lambda x: x.split("/")[0], arrowIDs)) + [
                    arrowIDs[-1].split("/")[-1]])
                entryExitRoutes[entryExitRouteID] = {
                    "startWaypointID": list(filter(lambda x: x["arrowID"] == arrowIDs[0], borderPoints))[0]["prevWaypointID"],
                    "goalWaypointID": list(filter(lambda x: x["arrowID"] == arrowIDs[-1], borderPoints))[0]["nextWaypointID"],
                    "arrowIDs": arrowIDs
                }

        return entryExitRoutes

    def isIentersected(self, ax, ay, bx, by, cx, cy, dx, dy):
        ta = (cx - dx) * (ay - cy) + (cy - dy) * (cx - ax)
        tb = (cx - dx) * (by - cy) + (cy - dy) * (cx - bx)
        tc = (ax - bx) * (cy - ay) + (ay - by) * (ax - cx)
        td = (ax - bx) * (dy - ay) + (ay - by) * (ax - dx)
        return tc * td <= 0 and ta * tb <= 0

    def __addStopPointToArrows(self):
        import sympy.geometry as sg
        for stopLine in self.__vectorMap.stopline.values():
            bpid = self.__vectorMap.line[stopLine["LID"]]["BPID"]
            fpid = self.__vectorMap.line[stopLine["LID"]]["FPID"]
            blat, blng = self.__vectorMap.point[bpid]["lat"], self.__vectorMap.point[bpid]["lng"]
            flat, flng = self.__vectorMap.point[fpid]["lat"], self.__vectorMap.point[fpid]["lng"]
            for arrowID, arrow in self.__arrows.items():
                for i in range(1, len(arrow["waypointIDs"])):
                    abpid = arrow["waypointIDs"][i-1]
                    afpid = arrow["waypointIDs"][i]
                    ablat, ablng = self.__vectorMap.point[abpid]["lat"], self.__vectorMap.point[abpid]["lng"]
                    aflat, aflng = self.__vectorMap.point[afpid]["lat"], self.__vectorMap.point[afpid]["lng"]

                    if self.isIentersected(blat, blng, flat, flng, ablat, ablng, aflat, aflng):
                        intersection = sg.intersection(
                            sg.Segment(sg.Point(blat, blng), sg.Point(flat, flng)),
                            sg.Segment(sg.Point(ablat, ablng), sg.Point(aflat, aflng))
                        )
                        stopPoints = self.__arrows[arrowID].get("stopPoints", [])
                        stopPoints.append({
                            "nextWaypointID": arrow["waypointIDs"][i],
                            "lat": float(intersection[0].x),
                            "lng": float(intersection[0].y),
                        })
                        self.__arrows[arrowID]["stopPoints"] = stopPoints

    def __pointInnerPolygon(self, point, polygon):
        def isLeft(p0, p1, p2):
            return (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1])

        windingNumber = 0
        polygon = tuple(polygon[:]) + (polygon[0],)
        for i in range(len(polygon) - 1):
            if polygon[i][1] <= point[1]:
                if polygon[i + 1][1] > point[1]:
                    if isLeft(polygon[i], polygon[i + 1], point) > 0:
                        windingNumber += 1
            else:
                if polygon[i + 1][1] <= point[1]:
                    if isLeft(polygon[i], polygon[i + 1], point) < 0:
                        windingNumber -= 1
        return False if 0 == windingNumber else True

    def __addNidToPoints(self):
        for pid in self.__vectorMap.point:
            self.__vectorMap.point[pid].update({"NIDS": []})

        for node in self.__vectorMap.node.values():
            self.__vectorMap.point[node["PID"]]["NIDS"].append(node["NID"])

    def __addLidToPoints(self):
        for pid in self.__vectorMap.point:
            self.__vectorMap.point[pid].update({
                "FLIDS": [],
                "BLIDS": []
            })

        for line in self.__vectorMap.line.values():
            self.__vectorMap.point[line["BPID"]]["FLIDS"].append(line["LID"])
            self.__vectorMap.point[line["FPID"]]["BLIDS"].append(line["LID"])

    def __getViewPoint(self):
        latMin = min(self.__vectorMap.point.values(), key=lambda x: x["lat"])["lat"]
        latMax = max(self.__vectorMap.point.values(), key=lambda x: x["lat"])["lat"]
        lngMin = min(self.__vectorMap.point.values(), key=lambda x: x["lng"])["lng"]
        lngMax = max(self.__vectorMap.point.values(), key=lambda x: x["lng"])["lng"]
        self.__viewData["viewPoint"]["lat"] = 0.5*(latMax + latMin)
        self.__viewData["viewPoint"]["lng"] = 0.5*(lngMax + lngMin)

    def getViewData(self):
        self.__viewData.update({
            "point": self.__vectorMap.point,
            "node": self.__vectorMap.node,
            "line": self.__vectorMap.line,
            "area": self.__vectorMap.area,
            "dtlane": self.__vectorMap.dtlane,
            "stopLine": self.__vectorMap.stopline,
            "intersection": self.__vectorMap.intersection,
            "arrows": self.__arrows,
            "toArrows": self.__toArrows,
            "fromArrows": self.__fromArrows,
            "intersections": self.__intersections,
        })
        return self.__viewData

    def dumpData(self, path="./"):
        with open(path+"arrow.json", "w") as f:
            json.dump({
                "arrows": self.__arrows,
                "toArrows": self.__toArrows,
                "fromArrows": self.__fromArrows
            }, f, indent="  ")
        with open(path+"waypoint.json", "w") as f:
            json.dump({
                "waypoints": self.__waypoints
            }, f, indent="  ")
        with open(path+"intersection.json", "w") as f:
            json.dump({
                "intersections": self.__intersections
            }, f, indent="  ")



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
        directory="./", filename="mapViewer.html")


@app.route("/getViewData")
def getViewData():
    pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    vectorMap = VectorMap()
    vectorMap.load(pathDir)
    mapAutoring = MapAuthoring(vectorMap)
    return api_response(code=200, message=mapAutoring.getViewData())


@app.route("/dumpData")
def dumpData():
    pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    vectorMap = VectorMap()
    vectorMap.load(pathDir)
    mapAutoring = MapAuthoring(vectorMap)
    mapAutoring.dumpData()
    return api_response(code=200, message={"result": "done"})


if __name__ == '__main__':
    # pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    # vectorMap = VectorMap()
    # vectorMap.load(pathDir)
    # vectorMap.dumpData()
    app.run(debug=True)
