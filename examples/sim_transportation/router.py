#!/usr/bin/env python
# coding: utf-8

import eventlet
from sys import float_info
from argparse import ArgumentParser

from flask import Flask, jsonify, render_template
from flask_cors import CORS
from flask_mqtt import Mqtt
from flask_socketio import SocketIO

from config.env import env
from ams.maps import Waypoint, Arrow, Intersection
from ams.helpers import Target, Topic
from ams.nodes import \
    User, Vehicle, FleetManager, \
    SimTaxi, SimTaxiUser, SimTaxiFleet, \
    SimBus, SimBusUser, SimBusFleet, \
    TrafficSignal

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint

parser = ArgumentParser()
parser.add_argument("-W", "--path_waypoint_json", type=str,
                    default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument("-A", "--path_arrow_json", type=str,
                    default="../../res/arrow.json", help="arrow.json path")
parser.add_argument("-I", "--path_intersection_json", type=str,
                    default="../../res/intersection.json", help="intersection.json path")
args = parser.parse_args()

eventlet.monkey_patch()

app = Flask(__name__)
with app.app_context():
    app.waypoint = Waypoint()
    app.waypoint.load(args.path_waypoint_json)
    app.arrow = Arrow()
    app.arrow.load(args.path_arrow_json)
    app.intersection = Intersection()
    app.intersection.load(args.path_intersection_json)

    app.topics = {
        "sim_taxi_user": Topic.get_topic(
            from_target=Target.new_target(SimTaxiUser.__name__, None),
            categories=User.CONST.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        ),
        "sim_taxi": Topic.get_topic(
            from_target=Target.new_target(SimTaxi.__name__, None),
            categories=Vehicle.CONST.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        ),
        "sim_taxi_fleet": Topic.get_topic(
            from_target=Target.new_target(SimTaxiFleet.__name__, None),
            categories=FleetManager.CONST.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        ),
        "sim_bus_user": Topic.get_topic(
            from_target=Target.new_target(SimBusUser.__name__, None),
            categories=User.CONST.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        ),
        "sim_bus": Topic.get_topic(
            from_target=Target.new_target(SimBus.__name__, None),
            categories=Vehicle.CONST.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        ),
        "sim_bus_fleet": Topic.get_topic(
            from_target=Target.new_target(SimBusFleet.__name__, None),
            categories=FleetManager.CONST.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        ),
        "traffic_signal": Topic.get_topic(
            from_target=Target.new_target(TrafficSignal.__name__, None),
            categories=TrafficSignal.CONST.TOPIC.CATEGORIES.STATUS,
            use_wild_card=True
        )
    }

    pp(app.topics)

CORS(app)

app.config['MQTT_BROKER_URL'] = env["MQTT_BROKER_HOST"]
app.config['MQTT_BROKER_PORT'] = int(env["MQTT_BROKER_PORT"])
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
    for topic in app.topics.values():
        mqtt.subscribe(topic)
    return render_template("index.html", title="ams", name="test_name")


@app.route("/getViewData")
def get_view_data():
    waypoint_ids = app.waypoint.get_waypoint_ids()
    waypoints = {}
    arrows = app.arrow.get_arrows()
    lat_min, lng_min = float_info.max, float_info.max
    lat_max, lng_max = 0.0, 0.0
    for waypoint_id in waypoint_ids:
        lat, lng = app.waypoint.get_latlng(waypoint_id)
        lat_min = min(lat_min, lat)
        lat_max = max(lat_max, lat)
        lng_min = min(lng_min, lng)
        lng_max = max(lng_max, lng)
        waypoints[waypoint_id] = {
            "geohash": app.waypoint.get_geohash(waypoint_id),
            "position": app.waypoint.get_position(waypoint_id)
        }
    return api_response(code=200, message={
        "viewPoint": {
            "lat": 0.5*(lat_max + lat_min),
            "lng": 0.5*(lng_max + lng_min)},
        "waypoints": waypoints,
        "arrows": arrows,
        "topics": app.topics
    })


@app.route("/requestFleetRelations")
def request_fleet_relations():
    return api_response(code=200, message={"result": "requested"})


@socketio.on('message')
def handle_message(message):
    print("handle_message", message)


@mqtt.on_topic(app.topics["sim_taxi_user"])
def handle_mqtt_on_sim_taxi_user_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["sim_taxi_user"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["sim_taxi"])
def handle_mqtt_on_sim_taxi_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["sim_taxi"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["sim_taxi_fleet"])
def handle_mqtt_on_sim_taxi_fleet_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["sim_taxi_fleet"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["sim_bus_user"])
def handle_mqtt_on_sim_bus_user_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["sim_bus_user"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["sim_bus"])
def handle_mqtt_on_sim_bus_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["sim_bus"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["sim_bus_fleet"])
def handle_mqtt_on_sim_bus_fleet_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["sim_bus_fleet"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["traffic_signal"])
def handle_mqtt_on_traffic_signal_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["traffic_signal"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


if __name__ == '__main__':
    socketio.run(app, host=env["AMS_HOST"], port=int(env["AMS_PORT"]), use_reloader=True, debug=True)
