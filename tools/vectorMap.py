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
                "lng": None
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
        degree = float(degreeStr) + float(mmss[:2])/60.0 + float(".".join([mmss[2:4], mmss[4:]]))/3600.0
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
        return points

    def __getViewPoint(self):
        latMin = min(self.point.values(), key=lambda x: x["lat"])["lat"]
        latMax = max(self.point.values(), key=lambda x: x["lat"])["lat"]
        lngMin = min(self.point.values(), key=lambda x: x["lng"])["lng"]
        lngMax = max(self.point.values(), key=lambda x: x["lng"])["lng"]
        self.__viewData["viewPoint"]["lat"] = 0.5*(latMax + latMin)
        self.__viewData["viewPoint"]["lng"] = 0.5*(lngMax + lngMin)

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

    def dumpData(self, path="./"):
        with open(path+"arrow.json", "w") as f:
            json.dump({
                "arrows": self.__arrows,
                "toArrows": self.__toArrows,
                "fromArrows": self.__fromArrows
            }, f, indent="  ")
        with open(path+"spot.json", "w") as f:
            json.dump({
                "spots": self.__spots
            }, f, indent="  ")
        with open(path+"waypoint.json", "w") as f:
            json.dump({
                "spots": self.__spots
            }, f, indent="  ")
        # with open(path+"intersection.json", "w") as f:
        #     json.dump({
        #         "intersections": self.intersection
        #     }, f, indent="  ")



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


if __name__ == '__main__':
    # pathDir = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1]+"/"
    # vectorMap = VectorMap()
    # vectorMap.load(pathDir)
    # vectorMap.dumpData()
    app.run(debug=True)
