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

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint

eventlet.monkey_patch()

app = Flask(__name__)

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
    return render_template("index.html", title="ams", name="test_name")


@app.route("/getGraph")
def get_node_graph():
    with open("./graph.dot", "r") as f:
        graph_dot = f.read()
    return api_response(200, {"dot": graph_dot})


@app.route("/getClassGraph")
def get_node_class_graph():
    with open("./class_graph.dot", "r") as f:
        graph_dot = f.read()
    return api_response(200, {"dot": graph_dot})


if __name__ == '__main__':
    socketio.run(app, host=env["AMS_HOST"], port=int(env["AMS_PORT"]), use_reloader=True, debug=True)
