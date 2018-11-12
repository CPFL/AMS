#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser


def append_lane(input_lane_json_path, output_lane_json_path, waypoint_ids_csv):
    with open(input_lane_json_path, "r") as f:
        input_lanes = json.load(f)

    waypoint_ids = waypoint_ids_csv.split(",")
    lane_code = "_".join([waypoint_ids[0], waypoint_ids[-1]])
    input_lanes["lanes"][lane_code] = {
        "laneCode": lane_code,
        "waypointIDs": waypoint_ids
    }
    with open(output_lane_json_path, "w") as f:
        json.dump(input_lanes, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-ILJP", "--input_lane_json_path", type=str, required=True, help="input lane.json file path")
    parser.add_argument("-OLJP", "--output_lane_json_path", type=str, default=None, help="output lane.json file path")
    parser.add_argument(
        "-WIDC", "--waypoint_ids_csv", type=str, default=None, help="waypoint_ids as csv")
    args = parser.parse_args()

    output_lane_json_path = args.output_lane_json_path
    if args.output_lane_json_path is None:
        output_lane_json_path = args.input_lane_json_path

    append_lane(
        args.input_lane_json_path, output_lane_json_path, args.waypoint_ids_csv
    )
