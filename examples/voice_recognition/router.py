#!/usr/bin/env python
# coding: utf-8

import eventlet
from sys import float_info
from argparse import ArgumentParser

from flask import Flask, jsonify, render_template, request
from flask_cors import CORS
from flask_mqtt import Mqtt
from flask_socketio import SocketIO

from config.env import env
from ams import Waypoint, Arrow, Intersection, Target, Topic
from ams.nodes import SimTaxi, SimTaxiUser, SimTaxiFleet, TrafficSignal, User, Vehicle, FleetManager, AutowareVoiceRecognition
from ams.structures import AUTOWARE_VOICE_RECOGNITION
# from ams.messages import FleetStatus

from pprint import PrettyPrinter


pp = PrettyPrinter(indent=2).pprint

parser = ArgumentParser()
parser.add_argument("-W", "--path_waypoint_json", type=str,
                    default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument("-A", "--path_arrow_json", type=str,
                    default="../../res/arrow.json", help="arrow.json path")
parser.add_argument("-I", "--path_intersection_json", type=str,
                    default="../../res/intersection.json", help="intersection.json path")
parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
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

    app.topics = {}

    topic = Topic()
    topic.set_targets(Target.new_target(None, SimTaxiUser.__name__), None)
    topic.set_categories(User.CONST.TOPIC.CATEGORIES.STATUS)
    app.topics["user"] = topic.get_path(use_wild_card=True)

    topic = Topic()
    topic.set_targets(Target.new_target(None, SimTaxi.__name__), None)
    topic.set_categories(Vehicle.CONST.TOPIC.CATEGORIES.STATUS)
    app.topics["vehicle"] = topic.get_path(use_wild_card=True)

    topic = Topic()
    topic.set_targets(Target.new_target(None, SimTaxiFleet.__name__), None)
    topic.set_categories(FleetManager.CONST.TOPIC.CATEGORIES.STATUS)
    app.topics["fleet_manager"] = topic.get_path(use_wild_card=True)

    topic = Topic()
    topic.set_targets(Target.new_target(None, TrafficSignal.__name__), None)
    topic.set_categories(TrafficSignal.CONST.TOPIC.CATEGORIES.STATUS)
    app.topics["traffic_signal"] = topic.get_path(use_wild_card=True)

    topicPubVoiceRecognition = Topic()
    topicPubVoiceRecognition.set_targets(Target.new_target(None, "voice"),
                                         Target.new_target(args.id, AutowareVoiceRecognition.__name__))
    topicPubVoiceRecognition.set_categories(AUTOWARE_VOICE_RECOGNITION.TOPIC.CATEGORIES.VOICE)

    topicPubCarStateToVoiceRecognition = "/voice/AMS_service_client/*/*/*"

    pp(app.topics)

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
            "position": dict(zip(["x", "y", "z"], app.waypoint.get_xyz(waypoint_id)))
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
    # topic = Topic()
    # topic.set_root(FleetManager.TOPIC.SUBSCRIBE)
    # topic.set_message(fleet_manager_message)
    # message = topic.get_template()
    # message["action"] = FleetManager.ACTION.PUBLISH_RELATIONS
    # mqtt.publish(topic.root, topic.serialize(message))
    return api_response(code=200, message={"result": "requested"})


@app.route("/voice", methods=["POST"])
def request_voice_recognition():
    if request.method == "POST":
        # print(request.json)
        message_voice_recognition = dict()
        if request.json["type"] == AUTOWARE_VOICE_RECOGNITION.VOICE_TYPE.CAR_CONTROL:
            message_voice_recognition["type"] = AUTOWARE_VOICE_RECOGNITION.VOICE_TYPE.CAR_CONTROL
            message_voice_recognition["details"] = dict()
            if request.json["details"] == "start":
                message_voice_recognition["details"]["state"] = AUTOWARE_VOICE_RECOGNITION.STATE.MOVE
            elif request.json["details"] == "stop":
                message_voice_recognition["details"]["state"] = AUTOWARE_VOICE_RECOGNITION.STATE.STOP
            elif request.json["details"] == "accelerate":
                message_voice_recognition["details"]["state"] = AUTOWARE_VOICE_RECOGNITION.STATE.ACCELERATE
            elif request.json["details"] == "decelerate":
                message_voice_recognition["details"]["state"] = AUTOWARE_VOICE_RECOGNITION.STATE.DECELERATE
        elif request.json["type"] == AUTOWARE_VOICE_RECOGNITION.VOICE_TYPE.DESTINATION_SET:
            message_voice_recognition["type"] = AUTOWARE_VOICE_RECOGNITION.VOICE_TYPE.DESTINATION_SET
            message_voice_recognition["details"] = dict()
            # 9701, 9397, 8912, 10358
            if request.json["details"] == "A":
                message_voice_recognition["details"]["waypoint"] = "9703"
            elif request.json["details"] == "B":
                message_voice_recognition["details"]["waypoint"] = "9397"
            elif request.json["details"] == "C":
                message_voice_recognition["details"]["waypoint"] = "8912"
            elif request.json["details"] == "D":
                message_voice_recognition["details"]["waypoint"] = "10358"



        else:
            return api_response(code=500, message="")

        mqtt.publish(topicPubVoiceRecognition.get_path(), topicPubVoiceRecognition.serialize(message_voice_recognition))

        return api_response(code=200, message={"result": True})

    return api_response(code=500, message="")



@socketio.on('message')
def handle_message(message):
    print("handle_message", message)


@mqtt.on_topic(app.topics["user"])
def handle_mqtt_on_user_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["user"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["vehicle"])
def handle_mqtt_on_vehicle_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["vehicle"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")

    if message["state"] == "stop" or message["state"] == "discharge":
        voice_generation_message = dict()
        voice_generation_message["type"] = "vehicle_state"
        voice_generation_message["details"] = "stop"
        mqtt.publish(topicPubCarStateToVoiceRecognition, Topic().serialize(voice_generation_message))


@mqtt.on_topic(app.topics["traffic_signal"])
def handle_mqtt_on_traffic_signal_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["traffic_signal"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


@mqtt.on_topic(app.topics["fleet_manager"])
def handle_mqtt_on_fleet_manager_status_message(_client, _userdata, mqtt_message):
    message = mqtt_message.payload.decode("utf-8")
    socketio.emit(
        app.topics["fleet_manager"], data={"topic": mqtt_message.topic, "message": message}, namespace="/ams")


if __name__ == '__main__':
    # socketio.run(app, host=env["AMS_HOST"], port=int(env["AMS_PORT"]), use_reloader=True, debug=True)
    socketio.run(app, host="0.0.0.0", port=int(env["AMS_PORT"]), use_reloader=True, debug=True)
