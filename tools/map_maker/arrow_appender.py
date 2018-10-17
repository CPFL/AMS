#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser


def append_arrow(input_arrow_json_path, output_arrow_json_path, waypoint_ids_csv):
    with open(input_arrow_json_path, "r") as f:
        input_arrows = json.load(f)

    waypoint_ids = waypoint_ids_csv.split(",")
    arrow_code = "_".join([waypoint_ids[0], waypoint_ids[-1]])
    input_arrows["arrows"][arrow_code] = {
        "arrowCode": arrow_code,
        "waypointIDs": waypoint_ids
    }
    with open(output_arrow_json_path, "w") as f:
        json.dump(input_arrows, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-IAJP", "--input_arrow_json_path", type=str, required=True, help="input arrow.json file path")
    parser.add_argument("-OAJP", "--output_arrow_json_path", type=str, default=None, help="output arrow.json file path")
    parser.add_argument(
        "-WIDC", "--waypoint_ids_csv", type=str, default=None, help="waypoint_ids as csv")
    args = parser.parse_args()

    output_arrow_json_path = args.output_arrow_json_path
    if args.output_arrow_json_path is None:
        output_arrow_json_path = args.input_arrow_json_path

    append_arrow(
        args.input_arrow_json_path, output_arrow_json_path, args.waypoint_ids_csv
    )
