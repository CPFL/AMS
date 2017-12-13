#!/usr/bin/env python
# coding: utf-8

from flask import Flask, send_from_directory, jsonify
from flask_cors import CORS
from config.env import env
from user import User
from waypoint import Waypoint

flask = Flask(__name__)
CORS(flask)

with flask.app_context():
    flask.amsWaypoint = Waypoint()
    flask.amsWaypoint.load("../../res/waypoint.json")
    flask.amsUser = User("testUser", flask.amsWaypoint)


def api_response(code=200, message={}):
    response = jsonify(message)
    response.status_code = code
    return response


@flask.route('/')
def root():
    return send_from_directory(
        directory="./", filename="index.html")


@flask.route("/getViewData")
def getViewData():
    waypoints = flask.amsWaypoint.getWaypoints()
    minLat = min(waypoints.items(), key=lambda x: x[1]["lat"])[1]["lat"]
    maxLat = max(waypoints.items(), key=lambda x: x[1]["lat"])[1]["lat"]
    minLng = min(waypoints.items(), key=lambda x: x[1]["lng"])[1]["lng"]
    maxLng = max(waypoints.items(), key=lambda x: x[1]["lng"])[1]["lng"]
    stopSpots = dict(filter(lambda x: x[0] in [
        "8910", "8911", "8912", "8913", "8914", "8915", "8916", "8917", "8918", "8919", "8920", "8921", "8922", "8923", "8924", "8925", "8926",
        "9362", "9363", "9364", "9365", "9366", "9367", "9368", "9369", "9370", "9371", "9372", "9373", "9374", "9375", "9376", "9377",
        "8883", "8884", "8885", "8886", "8887", "8888", "8889", "8890", "8891", "8892", "8893", "8894", "8895", "8896", "8897",
        "9392", "9393", "9394", "9395", "9396", "9397", "9398", "9399", "9400", "9401", "9402", "9403", "9404",
        # "9875", "9876", "9877", "9878", "9879", "9880", "9881", "9882", "9883", "9884", "9885", "9886", "9887",
        # "9922", "9923", "9924", "9925", "9926", "9927", "9928", "9929", "9930",
        # "9930", "9931", "9932", "9933", "9934", "9935",
        "10350", "10351", "10352", "10353", "10354", "10355", "10356", "10357", "10358", "10359", "10360", "10361", "10362", "10363", "10364", "10365", "10366", "10367", "10368", "10369", "10370", "10371", "10372", "10373", "10374",
        "9697", "9698", "9699", "9700", "9701", "9702", "9703", "9704", "9705", "9706", "9707", "9708",
        "8936", "8937", "8938", "8939", "8940", "8941", "8942", "8943", "8944", "8945", "8946", "8947", "8948", "8949", "8950", "8951", "8952", "8953", "8954", "8955", "8956", "8957", "8958", "8959", "8960", "8961", "8962", "8963", "8964", "8965", "8966", "8967", "8968",
        # "9144", "9145", "9146", "9147", "9148", "9149", "9150", "9151",
    ], waypoints.items()))
    message = {
        "viewPoint": {"lat": 0.5*(minLat+maxLat), "lng": 0.5*(minLng+maxLng)},
        "stopSpots": waypoints  # stopSpots
    }
    return api_response(code=200, message=message)


@flask.route("/setStartSpot/<waypointID>")
def setStartSpot(waypointID):
    message = {}
    flask.amsUser.setStartWaypoint(waypointID)
    return api_response(code=200, message={"result": "set"})


@flask.route("/setGoalSpot/<waypointID>")
def setGoalSpot(waypointID):
    flask.amsUser.setGoalWaypoint(waypointID)
    return api_response(code=200, message={"result": "set"})


@flask.route("/sendRequest")
def sendRequest():
    flask.amsUser.start()
    return api_response(code=200, message={"result": "sent"})



if __name__ == '__main__':
    flask.run(host=env["AMS_CLI_HOST"], port=int(env["AMS_CLI_PORT"]), debug=True)
