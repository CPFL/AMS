#!/usr/bin/env python
# coding: utf-8

import eventlet
from flask import Flask, send_from_directory, jsonify
from flask_cors import CORS
from flask_mqtt import Mqtt
from flask_socketio import SocketIO

from config.env import env
from ams import Waypoint, Arrow, Intersection
from ams.nodes import Vehicle as VEHICLE
from ams.nodes import User as USER
from ams.nodes import TrafficSignal as TRAFFIC_SIGNAL
from ams.nodes import FleetManager as FLEET_MANAGER

eventlet.monkey_patch()

app = Flask(__name__)
with app.app_context():
    app.arrow = Arrow()
    app.arrow.load("../res/arrow.json")
    app.waypoint = Waypoint()
    app.waypoint.load("../res/waypoint.json")
    app.intersection = Intersection()
    app.intersection.load("../res/intersection.json")
    app.fleetManagerMessage = {}

CORS(app)

app.config['MQTT_BROKER_URL'] = env["MQTT_BROKER_HOST"]
app.config['MQTT_BROKER_PORT'] = int(env["MQTT_BROKER_PORT"])
# app.config['MQTT_KEEPALIVE'] = 5
# app.config['MQTT_TLS_ENABLED'] = False
mqtt = Mqtt(app)

app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)


def api_response(code=200, message=None):
    if message is None:
        response = jsonify({})
    else:
        response = jsonify(message)
    response.status_code = code
    return response


@app.route('/')
def root():
    mqtt.subscribe(USER.TOPIC.PUBLISH+"/#")
    mqtt.subscribe(VEHICLE.TOPIC.PUBLISH+"/#")
    mqtt.subscribe(TRAFFIC_SIGNAL.TOPIC.PUBLISH+"/#")
    mqtt.subscribe(FLEET_MANAGER.TOPIC.PUBLISH+"/#")
    print("socketio", socketio)
    # socketio.start_background_task(target=background, comment='秒が経過')
    return send_from_directory(
        directory="./", filename="index.html")


@app.route("/getViewData")
def get_view_data():
    waypoints = app.waypoint.get_waypoints()
    arrows = app.arrow.get_arrows()
    lat_min = min(waypoints.values(), key=lambda x: x["lat"])["lat"]
    lat_max = max(waypoints.values(), key=lambda x: x["lat"])["lat"]
    lng_min = min(waypoints.values(), key=lambda x: x["lng"])["lng"]
    lng_max = max(waypoints.values(), key=lambda x: x["lng"])["lng"]
    return api_response(code=200, message={
        "viewPoint": {
            "lat": 0.5*(lat_max + lat_min),
            "lng": 0.5*(lng_max + lng_min)},
        "waypoints": waypoints,
        "arrows": arrows,
        "topics": {
            "user": USER.TOPIC.PUBLISH,
            "vehicle": VEHICLE.TOPIC.PUBLISH,
            "trafficSignal": TRAFFIC_SIGNAL.TOPIC.PUBLISH,
            "fleetManager": FLEET_MANAGER.TOPIC.PUBLISH
        }
    })


@app.route("/get_waypoints")
def get_waypoints():
    return api_response(code=200, message=app.waypoint.get_waypoints())


@app.route("/get_arrows")
def get_arrows():
    return api_response(code=200, message=app.arrow.get_arrows())


@app.route("/getIntersections")
def get_intersections():
    return api_response(code=200, message=app.intersection.getIntersections())


@app.route("/getFleetStatus")
def get_fleet_status():
    return api_response(code=200, message=app.fleetManagerMessage)


@socketio.on('message')
def handle_message(message):
    print("handle_message", message)


@mqtt.on_topic(USER.TOPIC.PUBLISH+"/#")
def handle_mqtt_ontopic_user_status(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(USER.TOPIC.PUBLISH, data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(VEHICLE.TOPIC.PUBLISH+"/#")
def handle_mqtt_ontopic_vehicle_status(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(VEHICLE.TOPIC.PUBLISH, data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(TRAFFIC_SIGNAL.TOPIC.PUBLISH+"/#")
def handle_mqtt_ontopic_traffic_signal_status(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        TRAFFIC_SIGNAL.TOPIC.PUBLISH, data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(FLEET_MANAGER.TOPIC.PUBLISH+"/#")
def handle_mqtt_ontopic_fleet_status(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    app.fleetManagerMessage = message


if __name__ == '__main__':
    socketio.run(app, host=env["AMS_HOST"], port=int(env["AMS_PORT"]), use_reloader=True, debug=True)


"""
mosquitto.configの変更　/etc/mosquitto/mosquitto.conf 
listener 1883

listener 9091 127.0.0.1
protocol websockets
を下部に追加


ヘッダー（index.htmlに記述）
<script src="https://cdnjs.cloudflare.com/ajax/libs/paho-mqtt/1.0.1/mqttws31.min.js" type="text/javascript"></script>

mainの記述

        // Create a client instance
        var client = new Paho.MQTT.Client("localhost", 9091, "clientId");
        // set callback handlers
        client.onConnectionLost = onConnectionLost;

        // connect the client
        client.connect({onSuccess:onConnect});


        // called when the client connects
        function onConnect() {
        // Once a connection has been made, make a subscription and send a message.
        console.log("mqtt onConnect");
        var message = new Paho.MQTT.Message("Hello");
        message.destinationName = "topic0";
        client.send(message);
        }

        // called when the client loses its connection
        function onConnectionLost(responseObject) {
        if (responseObject.errorCode !== 0) {
            console.log("onConnectionLost:"+responseObject.errorMessage);
        }
        }

        client.disconnect();
"""