#!/usr/bin/env python
# coding: utf-8

import csv
import json
import sys
import math
from copy import deepcopy
from argparse import ArgumentParser


def blend_map(
        from_waypoint_json_path, from_arrow_json_path,
        to_waypoint_json_path, to_arrow_json_path,
        output_waypoint_json_path, output_arrow_json_path,
):
    with open(from_waypoint_json_path, "r") as f:
        from_waypoints = json.load(f)
    with open(to_waypoint_json_path, "r") as f:
        to_waypoints = json.load(f)
    with open(from_arrow_json_path, "r") as f:
        from_arrows = json.load(f)
    with open(to_arrow_json_path, "r") as f:
        to_arrows = json.load(f)

    to_waypoints["waypoints"].update(from_waypoints["waypoints"])
    to_arrows["arrows"].update(from_arrows["arrows"])
    to_arrows["toArrows"].update(from_arrows["toArrows"])
    to_arrows["fromArrows"].update(from_arrows["fromArrows"])

    with open(output_waypoint_json_path, "w") as f:
        json.dump(to_waypoints, f, indent=" ")
    with open(output_arrow_json_path, "w") as f:
        json.dump(to_arrows, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-FWJP", "--from_waypoint_json_path", type=str, required=True, help="from waypoint.json file path")
    parser.add_argument("-FAJP", "--from_arrow_json_path", type=str, required=True, help="from arrow.json file path")
    parser.add_argument("-TWJP", "--to_waypoint_json_path", type=str, required=True, help="to waypoint.json file path")
    parser.add_argument("-TAJP", "--to_arrow_json_path", type=str, required=True, help="to arrow.json file path")
    parser.add_argument("-OWJP", "--output_waypoint_json_path", type=str, default=None, help="output waypoint.json file path")
    parser.add_argument("-OAJP", "--output_arrow_json_path", type=str, default=None, help="output arrow.json file path")
    args = parser.parse_args()

    output_waypoint_json_path = args.output_waypoint_json_path
    if args.output_waypoint_json_path is None:
        output_waypoint_json_path = args.to_waypoint_json_path

    output_arrow_json_path = args.output_arrow_json_path
    if args.output_arrow_json_path is None:
        output_arrow_json_path = args.to_arrow_json_path

    blend_map(
        args.from_waypoint_json_path, args.from_arrow_json_path,
        args.to_waypoint_json_path, args.to_arrow_json_path,
        output_waypoint_json_path, output_arrow_json_path)
