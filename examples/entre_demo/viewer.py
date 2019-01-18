#!/usr/bin/env python
# coding: utf-8

from os import listdir
from os.path import realpath
from argparse import ArgumentParser

from setproctitle import setproctitle
from flask import Flask, jsonify, render_template, send_from_directory

from ams.clients import MapsClient

app = Flask(__name__)
with app.app_context():
    app.maps_client = MapsClient()
    app.maps_client.load_waypoint_json_file("./static/maps/waypoint.json")
    app.maps_client.load_lane_json_file("./static/maps/lane.json")


def api_response(code=200, message={}):
    response = jsonify(message)
    response.status_code = code
    return response


@app.route('/')
def root():
    return render_template("index.html", title="ams", name="ams")


@app.route("/get/view/target/position")
def get_view_target_pose():
    waypoints = app.maps_client.waypoint.get_waypoints()
    view_target_position = {
        "x": 0.5 * (
                 min(map(lambda x: x["position"]["x"], waypoints.values())) +
                 max(map(lambda x: x["position"]["x"], waypoints.values()))
        ),
        "y": 0.5 * (
             min(map(lambda x: x["position"]["y"], waypoints.values())) +
             max(map(lambda x: x["position"]["y"], waypoints.values()))
        ),
        "z": 0.5 * (
             min(map(lambda x: x["position"]["z"], waypoints.values())) +
             max(map(lambda x: x["position"]["z"], waypoints.values()))
        )
    }
    return api_response(code=200, message=view_target_position)


@app.route("/get/ams/maps")
def get_ams_maps():
    ams_maps = {
        "waypoints": app.maps_client.waypoint.get_waypoints(),
        "lanes": app.maps_client.lane.get_lanes()
    }
    return api_response(code=200, message=ams_maps)


@app.route("/get/pcd/file/names")
def get_pcd_file_names():
    directory_path = realpath("./static/pcd")
    return api_response(code=200, message=listdir(directory_path))


@app.route('/get/pcd/file/<filename>')
def get_pcd_file(filename):
    directory_path = realpath("./static/pcd")
    return send_from_directory(
        directory=directory_path, filename=filename, as_attachment=True)


if __name__ == '__main__':
    setproctitle("ams_viewer")
    parser = ArgumentParser()
    parser.add_argument("-H", "--host", type=str, default="localhost", help="localhost")
    parser.add_argument("-P", "--port", type=int, default=5000, help="port number")
    args = parser.parse_args()
    app.run(host=args.host, port=args.port, use_reloader=True, debug=True)
