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
    route.set_waypoint(waypoint)
    route.set_arrow(arrow)
    # route.set_cost_function()
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

    routes = route.get_shortest_routes(startPoint, goalPoints, reverse=False)
    # routes = route.get_shortest_routes(startPoint, goalPoints)
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
