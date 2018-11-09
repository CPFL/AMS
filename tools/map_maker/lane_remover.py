#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser


def remove_lane(input_lane_json_path, output_lane_json_path, lane_code):
    with open(input_lane_json_path, "r") as f:
        input_lanes = json.load(f)

    input_lanes["lanes"].pop(lane_code)
    with open(output_lane_json_path, "w") as f:
        json.dump(input_lanes, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-ILJP", "--input_lane_json_path", type=str, required=True, help="input lane.json file path")
    parser.add_argument("-OLJP", "--output_lane_json_path", type=str, default=None, help="output lane.json file path")
    parser.add_argument("-LC", "--lane_code", type=str, default=None, help="lane_code")
    args = parser.parse_args()

    output_lane_json_path = args.output_lane_json_path
    if args.output_lane_json_path is None:
        output_lane_json_path = args.input_lane_json_path

    remove_lane(
        args.input_lane_json_path, output_lane_json_path, args.lane_code
    )
