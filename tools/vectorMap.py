import sys
import json
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint

ARROW_NAME_SEPARATER = "/"


class VectorMap(object):
    def __init__(self):
        self.__viewData = {
            "viewPoint": {
                "lat": None,
                "lng": None,
                "Bx": None,
                "Ly": None,
                "Hz": None,
            }
        }

    def load(self, pathDir):
        self.__pathPoint = pathDir + "point.csv"
        self.__pathNode = pathDir + "node.csv"
        self.__pathLine = pathDir + "line.csv"
        self.__pathLane = pathDir + "lane.csv"
        self.__pathDTLane = pathDir + "dtlane.csv"  # Road data-Center linear trajectory data
        self.__pathStopLine = pathDir + "stopline.csv"
        self.__pathArea = pathDir + "area.csv"
        self.__pathIntersection = pathDir + "intersection.csv"

        self.point = self.loadPoints(self.__pathPoint)
        self.node = self.loadCSV(self.__pathNode)
        self.line = self.loadCSV(self.__pathLine)
        self.lane = self.loadCSV(self.__pathLane)
        self.dtlane = self.loadCSV(self.__pathDTLane)
        self.stopline = self.loadCSV(self.__pathStopLine)
        self.area = self.loadCSV(self.__pathArea)
        self.intersection = self.loadCSV(self.__pathIntersection)

        self.__getViewPoint()

    @staticmethod
    def getDegreeFromDMMSS(dmmss):
        degreeStr, mmss = dmmss.split(".")
        degree = float(degreeStr) + float(mmss[:2])/60.0
        if 2 < len(mmss):
            degree += float(".".join([mmss[2:4], mmss[4:]]))/3600.0
        return degree

    @staticmethod
    def loadCSV(path):
        with open(path, "r") as f:
            lines = list(map(lambda x: x[:-1].split(","), f.readlines()))
            csvDict = dict(map(lambda x: (x[0], dict(zip(lines[0], x))), lines[1:]))
        return csvDict

    def loadPoints(self, path):
        points = self.loadCSV(path)

        for pid in points:
            points[pid]["lat"] = self.getDegreeFromDMMSS(points[pid]["B"])
            points[pid]["lng"] = self.getDegreeFromDMMSS(points[pid]["L"])
            points[pid]["Bx"] = float(points[pid]["Bx"])
            points[pid]["Ly"] = float(points[pid]["Ly"])
            points[pid]["H"] = float(points[pid]["H"])
        return points

    def __getViewPoint(self):
        latMin = min(self.point.values(), key=lambda x: x["lat"])["lat"]
        latMax = max(self.point.values(), key=lambda x: x["lat"])["lat"]
        lngMin = min(self.point.values(), key=lambda x: x["lng"])["lng"]
        lngMax = max(self.point.values(), key=lambda x: x["lng"])["lng"]
        bxMin = min(self.point.values(), key=lambda x: x["Bx"])["Bx"]
        bxMax = max(self.point.values(), key=lambda x: x["Bx"])["Bx"]
        lyMin = min(self.point.values(), key=lambda x: x["Ly"])["Ly"]
        lyMax = max(self.point.values(), key=lambda x: x["Ly"])["Ly"]
        hMin = min(self.point.values(), key=lambda x: x["H"])["H"]
        hMax = max(self.point.values(), key=lambda x: x["H"])["H"]
        self.__viewData["viewPoint"]["lat"] = 0.5*(latMax + latMin)
        self.__viewData["viewPoint"]["lng"] = 0.5*(lngMax + lngMin)
        self.__viewData["viewPoint"]["Bx"] = 0.5*(bxMax + bxMin)
        self.__viewData["viewPoint"]["Ly"] = 0.5*(lyMax + lyMin)
        self.__viewData["viewPoint"]["Hz"] = 0.5*(hMax + hMin)

    def getViewData(self):
        self.__viewData.update({
            "points": self.point,
            "nodes": self.node,
            "lines": self.line,
            "areas": self.area,
            "dtlanes": self.dtlane,
            "stopLines": self.stopline,
            "intersections": self.intersection,
        })
        return self.__viewData

    def get3DViewData(self):
        self.__viewData["points"] = []
        self.__viewData["lines"] = []
        self.__viewData["polylines"] = self.getAnyPolylines(self.point, self.line)
        for i, polyline in enumerate(self.__viewData["polylines"]):
            for j, vector in enumerate(polyline):
                self.__viewData["polylines"][i][j] = vector
        return self.__viewData

    @staticmethod
    def getAnyPolylines(nodes, links):
        linkIDs = list(links.keys())
        storedLinkIDs = []
        polylines = []
        while 0 < len(linkIDs):
            polyline = []
            nextLinkID = linkIDs[0]
            while nextLinkID in linkIDs:
                bpid = links[nextLinkID]["BPID"]
                fpid = links[nextLinkID]["FPID"]
                polyline.append({
                    "x": nodes[bpid]["Bx"],
                    "y": nodes[bpid]["Ly"],
                    "z": nodes[bpid]["H"]
                })
                storedLinkIDs.append(nextLinkID)
                linkIDs.remove(nextLinkID)
                nextLinkID = links[nextLinkID]["FLID"]
            polyline.append({
                "x": nodes[fpid]["Bx"],
                "y": nodes[fpid]["Ly"],
                "z": nodes[fpid]["H"]
            })
            polylines.append(polyline)
        return polylines


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
        directory="./", filename="vectorMapViewer.html")


@app.route("/getViewData")
def getViewData():
    pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    vectorMap = VectorMap()
    vectorMap.load(pathDir)
    return api_response(code=200, message=vectorMap.getViewData())


@app.route("/get3DViewData")
def get3DViewData():
    pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    vectorMap = VectorMap()
    vectorMap.load(pathDir)
    return api_response(code=200, message=vectorMap.get3DViewData())


if __name__ == '__main__':
    # pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    # vectorMap = VectorMap()
    # vectorMap.load(pathDir)
    # vectorMap.dumpData()
    app.run(debug=True)
