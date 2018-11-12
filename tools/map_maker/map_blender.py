#!/usr/bin/env python
# coding: utf-8

import csv
import json
import sys
import math
from copy import deepcopy
from argparse import ArgumentParser


def blend_map(
        from_waypoint_json_path, from_lane_json_path,
        to_waypoint_json_path, to_lane_json_path,
        output_waypoint_json_path, output_lane_json_path,
):
    with open(from_waypoint_json_path, "r") as f:
        from_waypoints = json.load(f)
    with open(to_waypoint_json_path, "r") as f:
        to_waypoints = json.load(f)
    with open(from_lane_json_path, "r") as f:
        from_lanes = json.load(f)
    with open(to_lane_json_path, "r") as f:
        to_lanes = json.load(f)

    to_waypoints["waypoints"].update(from_waypoints["waypoints"])
    to_lanes["lanes"].update(from_lanes["lanes"])
    to_lanes["toLanes"].update(from_lanes["toLanes"])
    to_lanes["fromLanes"].update(from_lanes["fromLanes"])

    with open(output_waypoint_json_path, "w") as f:
        json.dump(to_waypoints, f, indent=" ")
    with open(output_lane_json_path, "w") as f:
        json.dump(to_lanes, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-FWJP", "--from_waypoint_json_path", type=str, required=True, help="from waypoint.json file path")
    parser.add_argument("-FLJP", "--from_lane_json_path", type=str, required=True, help="from lane.json file path")
    parser.add_argument("-TWJP", "--to_waypoint_json_path", type=str, required=True, help="to waypoint.json file path")
    parser.add_argument("-TLJP", "--to_lane_json_path", type=str, required=True, help="to lane.json file path")
    parser.add_argument("-OWJP", "--output_waypoint_json_path", type=str, default=None, help="output waypoint.json file path")
    parser.add_argument("-OLJP", "--output_lane_json_path", type=str, default=None, help="output lane.json file path")
    args = parser.parse_args()

    output_waypoint_json_path = args.output_waypoint_json_path
    if args.output_waypoint_json_path is None:
        output_waypoint_json_path = args.to_waypoint_json_path

    output_lane_json_path = args.output_lane_json_path
    if args.output_lane_json_path is None:
        output_lane_json_path = args.to_lane_json_path

    blend_map(
        args.from_waypoint_json_path, args.from_lane_json_path,
        args.to_waypoint_json_path, args.to_lane_json_path,
        output_waypoint_json_path, output_lane_json_path)
