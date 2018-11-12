#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser


def merge_lanes(input_lane_json_path, output_lane_json_path, lane_code_csv):
    with open(input_lane_json_path, "r") as f:
        input_lanes = json.load(f)

    lane_codes = lane_code_csv.split(",")

    merged_lane_code = "_".join([lane_codes[0].split("_")[0], lane_codes[-1].split("_")[1]])
    merged_waypoint_ids = []
    for waypoint_ids in map(lambda x: input_lanes["lanes"][x]["waypointIDs"], lane_codes):
        if 0 < len(merged_waypoint_ids):
            merged_waypoint_ids.pop()
        merged_waypoint_ids.extend(waypoint_ids)
    input_lanes["lanes"][merged_lane_code] = {
        "laneCode": merged_lane_code,
        "waypointIDs": merged_waypoint_ids
    }
    with open(output_lane_json_path, "w") as f:
        json.dump(input_lanes, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-ILJP", "--input_lane_json_path", type=str, required=True, help="input lane.json file path")
    parser.add_argument("-OLJP", "--output_lane_json_path", type=str, default=None, help="output lane.json file path")
    parser.add_argument("-LCC", "--lane_code_csv", type=str, default=None, help="lane_codes as csv")
    args = parser.parse_args()

    output_lane_json_path = args.output_lane_json_path
    if args.output_lane_json_path is None:
        output_lane_json_path = args.input_lane_json_path

    merge_lanes(
        args.input_lane_json_path, output_lane_json_path, args.lane_code_csv
    )
