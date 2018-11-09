#!/usr/bin/env python
# coding: utf-8

import csv
import json
import sys
import math
from copy import deepcopy
from argparse import ArgumentParser

from ams.structures import LANE


def reconnect_lanes(input_lane_json_path, output_lane_json_path):
    with open(input_lane_json_path, "r") as f:
        lanes = json.load(f)["lanes"]

    to_lanes = {}
    from_lanes = {}
    for lane_code in lanes:
        waypoint_id1, waypoint_id2 = lane_code.split(LANE.DELIMITER)
        lane_codes = list(lanes.keys())
        to_lanes[lane_code] = list(filter(
            lambda x: waypoint_id2 == x.split(LANE.DELIMITER)[0] and waypoint_id1 != x.split(LANE.DELIMITER)[1],
            lane_codes))
        from_lanes[lane_code] = list(filter(
            lambda x: waypoint_id1 == x.split(LANE.DELIMITER)[1] and waypoint_id2 != x.split(LANE.DELIMITER)[0],
            lane_codes))

    output_lanes = {
        "lanes": lanes,
        "toLanes": to_lanes,
        "fromLanes": from_lanes
    }

    with open(output_lane_json_path, "w") as f:
        json.dump(output_lanes, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument(
        "-ILJP", "--input_lane_json_path", type=str, required=True, help="input lane.json file path")
    parser.add_argument(
        "-OLJP", "--output_lane_json_path", type=str, required=True, help="output lane.json file path")
    args = parser.parse_args()

    output_lane_json_path = args.output_lane_json_path
    if args.output_lane_json_path is None:
        output_lane_json_path = args.to_lane_json_path

    reconnect_lanes(args.input_lane_json_path, output_lane_json_path)
