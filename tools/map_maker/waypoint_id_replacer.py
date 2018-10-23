#!/usr/bin/env python
# coding: utf-8

import json
from copy import deepcopy
from argparse import ArgumentParser

from ams import logger


def replace_waypoint_ids(
        input_waypoint_json_path, input_arrow_json_path,
        output_waypoint_json_path, output_arrow_json_path,
        from_waypoint_id, to_waypoint_id
):
    with open(input_waypoint_json_path, "r") as f:
        input_waypoints = json.load(f)
    with open(input_arrow_json_path, "r") as f:
        input_arrows = json.load(f)

    input_waypoints["waypoints"].pop(from_waypoint_id)
    output_arrows = deepcopy(input_arrows)
    for arrow_code, arrow in input_arrows["arrows"].items():
        output_arrow = output_arrows["arrows"].pop(arrow_code)
        output_arrow["waypointIDs"] = [to_waypoint_id if waypoint_id == from_waypoint_id else waypoint_id for waypoint_id in arrow["waypointIDs"]]
        output_arrow["arrowCode"] = "_".join([to_waypoint_id if waypoint_id == from_waypoint_id else waypoint_id for waypoint_id in arrow_code.split("_")])
        output_arrows["arrows"][output_arrow["arrowCode"]] = output_arrow
        if from_waypoint_id in arrow["waypointIDs"]:
            logger.info("in: {}, out: {}, arrow: {}".format(arrow["waypointIDs"], output_arrow["waypointIDs"], output_arrow))

    with open(output_waypoint_json_path, "w") as f:
        json.dump(input_waypoints, f, indent=" ")
    with open(output_arrow_json_path, "w") as f:
        json.dump(output_arrows, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-IWJP", "--input_waypoint_json_path", type=str, required=True, help="input waypoint.json file path")
    parser.add_argument("-IAJP", "--input_arrow_json_path", type=str, required=True, help="input arrow.json file path")
    parser.add_argument("-OWJP", "--output_waypoint_json_path", type=str, default=None, help="output waypoint.json file path")
    parser.add_argument("-OAJP", "--output_arrow_json_path", type=str, default=None, help="output arrow.json file path")
    parser.add_argument(
        "-FWID", "--from_waypoint_id", type=str, default=None, help="from waypoint_id(this waypoint_id is abolished.)")
    parser.add_argument(
        "-TWID", "--to_waypoint_id", type=str, default=None, help="to waypoint_id")
    args = parser.parse_args()

    output_waypoint_json_path = args.output_waypoint_json_path
    if args.output_waypoint_json_path is None:
        output_waypoint_json_path = args.input_waypoint_json_path

    output_arrow_json_path = args.output_arrow_json_path
    if args.output_arrow_json_path is None:
        output_arrow_json_path = args.input_arrow_json_path

    replace_waypoint_ids(
        args.input_waypoint_json_path, args.input_arrow_json_path,
        output_waypoint_json_path, output_arrow_json_path,
        args.from_waypoint_id, args.to_waypoint_id
    )
