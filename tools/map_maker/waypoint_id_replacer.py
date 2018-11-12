#!/usr/bin/env python
# coding: utf-8

import json
from copy import deepcopy
from argparse import ArgumentParser

from ams import logger


def replace_waypoint_ids(
        input_waypoint_json_path, input_lane_json_path,
        output_waypoint_json_path, output_lane_json_path,
        from_waypoint_id, to_waypoint_id
):
    with open(input_waypoint_json_path, "r") as f:
        input_waypoints = json.load(f)
    with open(input_lane_json_path, "r") as f:
        input_lanes = json.load(f)

    input_waypoints["waypoints"].pop(from_waypoint_id)
    output_lanes = deepcopy(input_lanes)
    for lane_code, lane in input_lanes["lanes"].items():
        output_lane = output_lanes["lanes"].pop(lane_code)
        output_lane["waypointIDs"] = [to_waypoint_id if waypoint_id == from_waypoint_id else waypoint_id for waypoint_id in lane["waypointIDs"]]
        output_lane["laneCode"] = "_".join([to_waypoint_id if waypoint_id == from_waypoint_id else waypoint_id for waypoint_id in lane_code.split("_")])
        output_lanes["lanes"][output_lane["laneCode"]] = output_lane
        if from_waypoint_id in lane["waypointIDs"]:
            logger.info("in: {}, out: {}, lane: {}".format(lane["waypointIDs"], output_lane["waypointIDs"], output_lane))

    with open(output_waypoint_json_path, "w") as f:
        json.dump(input_waypoints, f, indent=" ")
    with open(output_lane_json_path, "w") as f:
        json.dump(output_lanes, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-IWJP", "--input_waypoint_json_path", type=str, required=True, help="input waypoint.json file path")
    parser.add_argument("-ILJP", "--input_lane_json_path", type=str, required=True, help="input lane.json file path")
    parser.add_argument("-OWJP", "--output_waypoint_json_path", type=str, default=None, help="output waypoint.json file path")
    parser.add_argument("-OLJP", "--output_lane_json_path", type=str, default=None, help="output lane.json file path")
    parser.add_argument(
        "-FWID", "--from_waypoint_id", type=str, default=None, help="from waypoint_id(this waypoint_id is abolished.)")
    parser.add_argument(
        "-TWID", "--to_waypoint_id", type=str, default=None, help="to waypoint_id")
    args = parser.parse_args()

    output_waypoint_json_path = args.output_waypoint_json_path
    if args.output_waypoint_json_path is None:
        output_waypoint_json_path = args.input_waypoint_json_path

    output_lane_json_path = args.output_lane_json_path
    if args.output_lane_json_path is None:
        output_lane_json_path = args.input_lane_json_path

    replace_waypoint_ids(
        args.input_waypoint_json_path, args.input_lane_json_path,
        output_waypoint_json_path, output_lane_json_path,
        args.from_waypoint_id, args.to_waypoint_id
    )
