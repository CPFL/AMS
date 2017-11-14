#!/usr/bin/env python
# coding: utf-8

import eventlet
from flask import Flask, send_from_directory, jsonify
from flask_cors import CORS
from flask_mqtt import Mqtt
from flask_socketio import SocketIO

from waypoint import Waypoint
from arrow import Arrow
from intersection import Intersection
from const import CONST

eventlet.monkey_patch()

app = Flask(__name__)
with app.app_context():
    app.arrow = Arrow()
    app.arrow.load("../res/arrow.json")
    app.waypoint = Waypoint()
    app.waypoint.load("../res/waypoint.json")
    app.intersection = Intersection()
    app.intersection.load("../res/intersection.json")

CORS(app)

app.config['MQTT_BROKER_URL'] = 'localhost'
app.config['MQTT_BROKER_PORT'] = 1883
# app.config['MQTT_KEEPALIVE'] = 5
# app.config['MQTT_TLS_ENABLED'] = False
mqtt = Mqtt(app)

app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)


@app.route('/')
def root():
    mqtt.subscribe(CONST.TOPICS.USER_STATUS)
    mqtt.subscribe(CONST.TOPICS.VEHICLE_STATUS)
    mqtt.subscribe(CONST.TOPICS.TRAFFICSIGNAL_STATUS)
    mqtt.subscribe(CONST.TOPICS.FLEET_STATUS)
    print("socketio", socketio)
    # socketio.start_background_task(target=background, comment='秒が経過')
    return send_from_directory(
        directory="./", filename="index.html")


def api_response(code=200, message={}):
    response = jsonify(message)
    response.status_code = code
    return response


@app.route("/getViewData")
def getViewData():
    waypoints = app.waypoint.getWaypoints()
    arrows = app.arrow.getArrows()
    intersections = app.intersection.getIntersections()
    latMin = min(waypoints.values(), key=lambda x: x["lat"])["lat"]
    latMax = max(waypoints.values(), key=lambda x: x["lat"])["lat"]
    lngMin = min(waypoints.values(), key=lambda x: x["lng"])["lng"]
    lngMax = max(waypoints.values(), key=lambda x: x["lng"])["lng"]
    return api_response(code=200, message={
        "viewPoint": {
            "lat": 0.5*(latMax + latMin),
            "lng": 0.5*(lngMax + lngMin)},
        "waypoints": waypoints,
        "arrows": arrows,
        "intersections": intersections,
    })


@app.route("/getWaypoints")
def getWaypoints():
    return api_response(code=200, message=app.waypoint.getWaypoints())


@app.route("/getArrows")
def getArrows():
    return api_response(code=200, message=app.arrow.getArrows())


@app.route("/getIntersections")
def getIntersections():
    return api_response(code=200, message=app.intersections.getIntersections())


@app.route("/getFleetStatus")
def getFleetStatus():
    mqtt.publish(CONST.TOPICS.FLEET_STATUS, "")
    return api_response(code=200, message="")


@socketio.on('message')
def handle_message(message):
    print("handle_message", message)


@mqtt.on_topic(CONST.TOPICS.USER_STATUS)
def handle_mqtt_ontopic_user_status(client, userdata, message):
    data = message.payload.decode("utf-8")
    # print('Received message on topic {}: {}'
    #       .format(message.topic, message.payload.decode()))
    socketio.emit('userStatus', data={"data": data}, namespace="/ams")


@mqtt.on_topic(CONST.TOPICS.VEHICLE_STATUS)
def handle_mqtt_ontopic_vehicle_status(client, userdata, message):
    data = message.payload.decode("utf-8")
    # print('Received message on topic {}: {}'
    #       .format(message.topic, message.payload.decode()))
    socketio.emit('vehicleStatus', data={"data": data}, namespace="/ams")


@mqtt.on_topic(CONST.TOPICS.TRAFFICSIGNAL_STATUS)
def handle_mqtt_ontopic_traffic_signal_status(client, userdata, message):
    data = message.payload.decode("utf-8")
    # print('Received message on topic {}: {}'
    #       .format(message.topic, message.payload.decode()))
    socketio.emit('trafficSignalStatus', data={"data": data}, namespace="/ams")


@mqtt.on_topic(CONST.TOPICS.FLEET_STATUS)
def handle_mqtt_ontopic_fleet_status(client, userdata, message):
    data = message.payload.decode("utf-8")
    # print('Received message on topic {}: {}'
    #       .format(message.topic, message.payload.decode()))
    socketio.emit('fleetStatus', data={"data": data}, namespace="/ams")


if __name__ == '__main__':
    socketio.run(app, host='localhost', port=5000, use_reloader=True, debug=True)
